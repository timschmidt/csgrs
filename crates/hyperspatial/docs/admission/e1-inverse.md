# E1 closest-point geodetic inverse

Baseline: `87e357d5e`. Owner: Hyperspatial coordinate semantics over native
Hyperreal roots/trigonometry and Hyperlimit decisions. Relevant properties:
P1–P11, P18–P20, P26, P35–P39. Missing acceptance: the baseline has no ECEF
inverse API; axis/sphere, ordinary negative height, nonunique interior,
retained inverse construction and explicit pole-meridian tests cannot compile.

## Contract before implementation

The operation selects the closest surface point on a positive oblate, prolate
or spherical reference ellipsoid. Height is signed along its outward normal.
The coordinate frame/epoch is unchanged; the caller supplies the reference
surface. This is a selected inverse of the forward map, which can have multiple
normal-coordinate solutions in the deep interior.

Reject the centre and nonunique closest points with a typed ambiguity. For an
oblate spheroid these lie on the equatorial disk `z = 0`,
`rho < (a²-b²)/a`; for a prolate spheroid they lie on the rotation axis with
`abs(z) < (b²-a²)/b`. The cusp boundary has a unique closest point and remains
admitted. At a pole, require an explicit longitude convention or return a
typed ambiguity; retain the convention for subsequent local-frame construction.
Non-pole longitude is normalized to [-pi, pi).

Use exact represented algebraic/trigonometric expressions, including the
real-cubic/trigonometric branch of the Vermeille/Karney solution. No float,
fixed-iteration approximation, far-distance spherical substitution, epsilon
clamp, or unexplained inverse success is permitted. Every data-dependent branch
uses the context policy and retains its actual decision. Scalar/predicate and
orchestration failure remain typed. Existing inner-call time/heap limitations
continue to apply.

Sources: [Karney's primary implementation](https://github.com/geographiclib/geographiclib/blob/main/src/Geocentric.cpp),
[public inverse semantics](https://github.com/geographiclib/geographiclib/blob/main/include/GeographicLib/Geocentric.hpp),
and [the cited derivation, Appendix B](https://arxiv.org/abs/1102.1215v1).
The native adaptation retains the upstream MIT notice; no external runtime
dependency is introduced. Numerical certainty establishes the represented
equations, not physical Earth accuracy.

The result retains its Cartesian source and branch receipt. Returning that
immutable inverse result to ECEF may reuse its retained exact source under the
construction identity; independent tests must also evaluate latitude/height and
the forward equations to avoid letting this identity conceal paired errors.
Raw inputs and previously qualified results retain their original ancestry.

Acceptance combines independent axis/sphere identities, analytically constructed
normal offsets for oblate/prolate ellipses, both cubic discriminant branches,
the medial-axis boundary, exact tiny offsets, large distances, repeated local
composition, budgets/cancellation and source qualification. General-expression
comparisons use certified enclosures with stated test bounds, not native equality
tolerances. Full E1 status follows executed evidence, not this specification.

Add pinned `map_3d` 0.1.7 only as a development dependency. Compare ordinary
WGS84 forward/inverse/local outputs at explicit binary64 test boundaries. Match
the reference axes exactly to the oracle's represented parameters; distinguish
those rounded values from the exact defining WGS84 flattening. This comparison
supplements the independent mathematical cases and does not prove the model.
Reject nonfinite oracle ingress through Hyperreal's checked conversion.

The first comparison disproved the test's assumption that `map_3d`'s inverse
meets a 5e-13-radian latitude bound at 1,000 km altitude. Its own forward/inverse
probe fails that bound on 30 of the 175 retained cases (maximum about
1.2034e-9 radians at ±55 degrees); all height comparisons meet 5e-7 m. This is
an oracle qualification change, not a relaxation of the native contract.
Preserve the entire 175-case corpus and add independent certified forward
residual checks on every native inverse. Keep the original 5e-13-radian native
bound, and explicitly assert the known oracle discrepancies at high altitude.
Retain the full error inventory and a named regression for the worst case.

## Implemented surface and evidence

`EcefPosition::to_geodetic_closest` and `ecef_to_geodetic_batch` implement this
selection. `PoleMeridian` makes the axial convention explicit. `GeodeticSource`
retains the original Cartesian point and any selected pole meridian; the
latitude's own receipt inherits the inverse decisions. Only a certified
inverse may reuse the retained source identity when returning to ECEF. A
qualified approximate branch evaluates the forward expressions instead and
remains qualified. The normal dependency graph is unchanged.

The initial tests failed on the baseline with the missing method/types. Final
tests cover exact axis/sphere identities, ordinary negative heights, prolate
and oblate analytical normal offsets, both cubic discriminant branches,
unique cusp endpoints, a 1e-40-metre distinction at the medial boundary,
an axis coordinate beyond binary64 range and a general offset of 1e60 metres,
repeated inverse/forward/local consumption, tiny budgets, cancellation/recovery,
atomic batches, all eight public Real families in the equatorial-height
workflow, and inherited approximate branch evidence.

The independent normal-offset comparisons certify latitude/height enclosures
of 1e-40 radians/metres. The 175-case WGS84 oracle corpus uses its represented
binary64 axes, seven latitudes, five longitudes and five heights from -1 km to
1,000 km. Every native inverse passes the original 5e-13-radian latitude bound
and an independently evaluated 1e-25-metre forward residual; the comparison
also checks native forward/local display coordinates within 3e-8 m and heights
within 5e-7 m. These bounds are test evidence, not native predicate tolerances
or physical accuracy claims.

The full [map_3d error inventory](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-map3d-inverse-errors.csv)
retains all 175 cases of its own forward/inverse cycle. Its 30 latitude
discrepancies at 1,000 km are explicit expected oracle limitations in the
unchanged corpus, with a separate 55-degree regression. No native precision
bound was enlarged to accept them. The source's finite beta correction explains
why this binary64 comparator is not an exact inverse reference. Nonfinite
oracle outputs are rejected before constructing native coordinates.

## Validation and remaining admission work

| Check | Executed result |
| --- | --- |
| `bash crates/hyperspatial/scripts/checks.sh`, Rust 1.99 stable and 1.95 MSRV | 34 integration tests and the README doctest; all-target/default/all-feature builds and tests; Clippy and rustdoc with warnings denied; native dependency check; separate source consumer including the inverse; fuzz target formatting/build/lint. |
| `cargo check --locked --offline -p hyperspatial --lib --all-features --target wasm32-unknown-unknown` | Pass. |
| `python3 crates/hyperspatial/scripts/coverage.py` | 824/838 production lines (98.33%), 1151/1235 regions (93.20%), 102/102 functions; the 95% line floor passes. |
| Stable `cargo bench --locked --offline -p hyperspatial --bench inverse`; MSRV `cargo bench --locked --offline -p hyperspatial --benches` | All optimized rows complete with exact predicate certainty. |
| Seeded `forward_local` fuzz executable, expanded with inverse/independent-forward assertions | The same 918 six-byte cases and 1,000-run smoke complete under time/RSS limits. This is uninstrumented replay, not ASan or adaptive coverage-guided evidence. |
| `zizmor --pedantic --strict-collection .`, Zizmor 1.31.1 default offline audits | No findings after the workflow's optimized probe gate was extended. |

[Coverage data](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-inverse-coverage.json) retains the
source digests and instrumented denominator. [Fuzz replay output](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e1-inverse-fuzz-smoke.txt)
retains the missing-sanitizer/coverage warnings. [Inverse cost rows](../../benchmarks/2026-10-18-inverse.csv)
measure five warm trials of 100 complete inverse/evidence/display/source-return
workflows. [A fresh-process probe](../../benchmarks/2026-10-18-inverse-resources.json)
records peak RSS, executable size, compiler/profile and source digests. These
are cost observations, not a cross-library speedup, allocation count or proof
of a heap bound. The benchmark oracle runs outside the native measurement.

Open: 100% line/region and instrumented branch coverage, all internal scalar
dispatch classes/cache configurations, directed ASan/lexicase campaigns,
stronger scalar work/memory contracts, release packaging and WASM runtime
consumers. E2–E4 and A1–A5 remain active development work. No push or complete
repository pre-push campaign was performed in this increment.
