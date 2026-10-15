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

## Final local qualification

All 13 `native-v1` commands pass after the final implementation, documentation
and image edit. This includes formatting, all-target Clippy with warnings denied
under all features, no default features and curve-only; all-feature library and
integration tests (263 passed, one existing ignored image-corpus test); affected
minimal (38 passed) and curve-only (78 passed) suites; fuzz-target Clippy;
warning-free docs and one doctest; the original benchmark consumer (three
samples of eight operations); and the full renderer build. The overlapping
configurations total 380 passing test executions. The unchanged ignore is not
credited as a pass. No language binding directly exposes this constructor or
Gerber import, and no binding implementation or signature changes here.

`render-v1` failed to compile because the focused harness omitted its `hypermesh`
extern. `render-v2` retains the unchanged source driver, supplies that dependency,
and produces two identical 26,480-byte PNGs. The derived driver's only changes
are selecting the original rounded-rectangle fixture in `main` and rebasing the
unused font include path. Its patch, exact source hash and build inputs are
retained. The renderer functions, constants, fixture dimensions and 12 corner
segments remain unchanged. `docs/rounded_rectangle.png` intentionally changes
from its previous 29,473-byte artifact. This is focused regeneration, not a
successful full image-corpus regression.

## Corrected-workload resource probe

`cost-v1` passes its build and all 18 fresh-process samples: three repetitions
of three fixtures in each of two ring-cache states. Each sample has 128 warmup
and 128 measured operations. All 2,304 measured operations complete Certified,
materialize the boundary, verify every exact tangent endpoint and the expected
sample count, and drop the result within the timing interval. The one-chord
fixture also checks independently derived exact area. Misses cycle 128 widths
to exceed the 64-entry FIFO; this does not make scalar/kernel caches globally
cold. The n=8 benchmark and n=12 README inputs retain their original resolutions.

Median instrumented measurements follow. Time, allocation count and requested
bytes are per completed operation; peak live bytes is the largest live-byte
increase above the start of the measured 128-operation batch, with the median
across three samples. Rust global-allocator instrumentation adds overhead and
does not observe all native allocations or imply a heap-byte bound.

| Fixture | Ring cache | ms/op | allocations/op | requested bytes/op | batch peak extra live bytes |
| --- | --- | ---: | ---: | ---: | ---: |
| octagon | warm | 0.019223 | 193.000 | 10514.0 | 1320 |
| octagon | miss | 0.156462 | 936.898 | 115987.9 | 64776 |
| benchmark | warm | 0.554027 | 11929.000 | 705865.0 | 32680 |
| benchmark | miss | 8.278424 | 135885.398 | 7535948.2 | 422468 |
| readme | warm | 0.766030 | 12551.000 | 814953.0 | 58064 |
| readme | miss | 11.688429 | 161472.516 | 9179139.4 | 889977 |

The instrumented executable is 35,286,376 bytes; the all-feature CSGRS rlib is
10,020,198 bytes. Raw timings, allocation requests, retained-byte deltas,
per-process resource reports, hardware, toolchain, dependency lock and source
hashes are retained. These are focused costs of corrected outputs on a shared
host, not a speedup claim or a comparison against the old incorrect set. No
unfavorable measurement is removed. The original benchmark harness's checksum
is only a consumer smoke check; independent semantics come from the regression
and cost-probe assertions.

## Remaining scope

These trials include the unchanged 35 original pending
native files. Their separate admission, full CI/pre-push checks, external
package/release validation and the rest of the properties plan remain open.
No push has occurred.
