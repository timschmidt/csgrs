# Rounded-rectangle sampling fidelity

This record applies the acceptance contract in `properties.md` and
`docs/admission.md`. Its owning layer is CSGRS's sampled-profile constructor;
Hypercurve still owns contour/topology validation. Relevant properties are
P1, P8–P9, P17–P18, P20, P25–P27 and P35–P39. The preceding goal turn made
progress in commits `9d1f2fb5f`, `fd83df307` and `1ff12ff26`; the full plan
remains active.

The input domain is nonnegative width, height and radius. The radius clamps
to half the smaller dimension. Validated zero dimensions give canonical empty
geometry; zero radius or zero corner segments gives the sharp rectangle.
For positive radius and `n > 0`, the represented set has `n` equal-angle chords
per quarter-circle and every nonzero straight side. This is a sampled polygon,
not the analytic rounded rectangle. The clamp comparisons supply the facts
needed to omit only coincident endpoints, with their consumed certainty.

The implementation corrects geometry and therefore changes some vertex counts,
serialized artifacts and renderings. Capacity checking covers `4 * (n + 1)`
points. Numerical certification and manifoldness do not prove analytic fidelity.
There are still no internal total-work, scalar-growth, heap-byte or cancellation
bounds. Trials use external 1800-second wall and 8-GiB per-process address-space
containment, two Cargo jobs and offline dependencies.

## Preserved trials

- `baseline-v1`: on `1ff12ff26`, three of four tests fail. The one-chord 4×2,
  radius-1/2 polygon has area 4 instead of 15/2; clamped capsules have area 4
  instead of 6; the original 12×8 radius-2, eight-segment benchmark omits a
  tangent endpoint. The sharp/full-radius controls pass.
- `candidate-v1`: the original four regressions and all 49 curve library tests
  pass, with the original strict shape/gear catalog retained.
- `candidate-v2`: nine of ten expanded tests pass. The algebraic-area test
  incorrectly uses structural scalar equality for differently ordered sums;
  both sides represent `7 + sqrt(2)/2`. The failed test and all output remain.
- `candidate-v3`: all ten expanded tests pass after requiring a strict predicate
  proof of mathematical equality. No tolerance, policy or fixture is weakened.

The expanded oracle checks independent rational areas and tangent vertices,
algebraic and symbolic areas, the original benchmark and README resolutions
(including the README's binary64 literal ingress), clamping in both aspect
ratios, sharp/full-radius controls, and qualified approximate clamp decisions.
The public pipeline translates, subtracts a hole, restores the set by union,
subtracts it to empty, extrudes to signed volume 39/2, validates closed
manifoldness, and round-trips its exactly representable dyadic coordinates
through OBJ. Gerber obround import/export retains independently known straight
boundary points and inside/outside classifications. This is not a lossless
claim for arbitrary curved Gerber or OBJ geometry.

Broader post-edit qualification, the corrected image, and proportionate cost
evidence are pending. These trials include the unchanged 35 original pending
native files. Their separate admission, full CI/pre-push checks, external
package/release validation and the rest of the properties plan remain open.
No push has occurred.
