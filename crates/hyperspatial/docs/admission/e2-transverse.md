# Transverse Mercator chart and UTM selection admission

Base revision: `4b7b0ed4c`. Hyperspatial owns the coordinate model, chart,
prepared identity, branch decisions and provenance (P6, P8–P11, P17–P20).
Hyperreal owns the scalar arithmetic. The missing acceptance cases are native
spherical transverse projection/inversion and exact UTM zone selection.

The representation sweep found a missing boundary proof for fresh ingress at
`N0 + R*k*(pi/2-phi0)` with symbolic scale `k=pi^2*e`. Normalizing that expression
before comparison lost its recognizable endpoint and returned Unknown. Compare
the represented northing against the prepared physical strip endpoints first;
strict equality justifies reusing the exact normalized endpoint. Approximate
equality retains its qualification and must not replace the represented value.
The original failing six-byte case is retained in
`verification/2026-10-18-e2-transverse/transverse-representations-baseline.log`.

A later boundary regression reproduced an avoidable Unknown for an opaque
representation of |η|=1 at the internal hyperbolic-form switch. The equivalent
formulas overlap: after an unresolved comparison at 1, a separate certified
comparison with 1/2 can select a valid formula. This does not reinterpret
Unknown as an ordering, or suppress cancellation/resource errors. The failing
fixture remains in `internal-switch-baseline.log`. Retained transverse relations
use shared immutable storage, avoiding the enlarged `PoleMeridian` variant
reported by all-feature Clippy without changing its public convention API.

The [PROJ formulation](https://proj.org/en/stable/operations/projections/tmerc.html)
separates conformal-latitude conversion, the spherical transverse chart and
ellipsoidal rectification. This change implements the exact spherical component
as a reusable prerequisite; it does not substitute that model for ellipsoidal
TM/UTM. Ellipsoidal rectification, finite-model/remainder contracts, geodesics
and all later plan stages remain required. Karney's
[exact and series algorithms](https://arxiv.org/abs/1002.1417) are research
references for that remaining work.

For a genuine sphere of radius R, positive scale k, central meridian λ0 and
origin latitude φ0, define δ=λ−λ0 and

    ξ = atan2(sin φ, cos φ cos δ)
    η = asinh(cos φ sin δ / hypot(sin φ, cos φ cos δ))
    E = E0 + Rk η, N = N0 + Rk(ξ−φ0), H = h.

The chart includes −π/2≤δ≤π/2, −π/2≤φ≤π/2. Its two equatorial edges
(φ=0, δ=±π/2) are singular. At the geographic poles longitude selects a local
meridian but does not change the projected point; inverse pole selection uses
the existing explicit `PoleMeridian` convention. Nonpolar outside-chart input
is rejected. Longitude policy either rejects extra turns or explicitly wraps
before the chart check. The projected domain is −π/2≤ξ≤π/2 with finite η.
Height, reference frame, coordinate epoch and input qualification pass through.

Inverse equations are φ=atan2(sin ξ,hypot(sinh η,cos ξ)) and
δ=atan2(sinh η,cos ξ). Large |η| must use equivalent exponentially scaled
equations so that bounded outputs do not require constructing exp(|η|).
Read-only probes on the base revision confirmed exp(-10^320),
exp(-10^320+π) and exp(-10^320 e) return certified intervals at precision -128;
those standalone probes did not reproduce a failure. Composition subsequently
reproduced a default-stack abort in `atan(exp(-10^320))`, caused by 1065 nested
squares in the native deferred exponential kernel. The
[scalar admission](../../../hyperreal/docs/admission-negative-exp-tail.md)
records the precision-dependent bound and integer-log fallback repairs; no
stored exponential is rounded to zero. Baseline and fixed logs are retained.

Inverse latitude and longitude retain construction range evidence. A public
clone keeps it; extracting a raw angle relinquishes it. Approximate ancestry
must not become a strict chart certificate. Prepared-source replay must still
validate surface, frame and applicable chart conventions. Batches remain atomic;
Context checkpoints, batch length and receipt depth are not scalar time/heap
bounds. Unknown, exhausted, cancelled, invalid, ambiguous and singular outcomes
remain distinct typed errors.

UTM zone selection is a separate angular classification, not an ellipsoidal
projection. It accepts canonical longitude in [-180°,180°), latitude in
[-80°,84°), assigns the equator to the north, and implements the Norway and
Svalbard exceptions with exact boundary comparisons. Wrapping is an explicit
caller operation. Zone numbers are identifiers; all angular, length and scale
parameters remain `Real` backed. The source coordinate and decision receipt
must be retained. GeographicLib's
[UTM/UPS implementation](https://raw.githubusercontent.com/geographiclib/geographiclib/main/src/UTMUPS.cpp)
is a reference for the zone conventions (checked 2026-10-18); UPS is not an
implicit fallback.

Acceptance requires independent analytic coordinates, dev-only projection
oracles, both pole conventions and both singular edges, false origins,
longitude seams, prepared/reference/surface mismatch, huge finite eastings,
qualified ancestry, all 22 scalar families, cache configurations, downstream
ECEF/local consumption, cancellation/exhaustion, directed/ASan testing, coverage
and source consumers. UTM fixtures must exercise all 60 ordinary zones and
exact/near Norway, Svalbard, equator, coverage and antimeridian boundaries.

The implementation is retained with
[execution evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperspatial/verification/2026-10-18-e2-transverse/README.md).
The public APIs are `SphericalTransverseMercator`, its parameters and explicit
longitude policy, and `UtmSelection`/`UtmZone`/`Hemisphere`. The unpublished
`GeodeticSource` enum adds a transverse source variant; exhaustive consumers
must handle it. Shared longitude relation storage keeps public pole-convention
construction unchanged. No serialization or binding ABI is introduced.

The final stable and Rust 1.95.0 check suites each pass 154 test executions,
including both documented source consumers. Four cache configurations pass
300 executions and strict Clippy; final focused boundary checks also cover
±10^-500° hemisphere/coverage decisions. Production line coverage is 98.5804%;
all 201 instrumented functions execute, while branches remain unmeasured.
The final directed search completes all 176 cases in 22 buckets. The final ASan
campaign completes 28,142 runs in 901 seconds without a new artifact; a separate
final-selection campaign and all 15 direct regression tests also pass under
ASan. Native-scalar, WASM, benchmark and CI-audit evidence is in the bundle.

PROJ4RS 0.2.1 fails the unchanged 3e-7 m comparison at two equatorial inputs
because its spherical northing is about 0.300548 m instead of zero. Those
fixtures and the initial failure are retained; native results are checked
against the independent equatorial identity. Its inverse is tested separately
on validated native projections. No comparator failure is counted as an exact
completion, and no threshold or input was weakened to obtain acceptance.

Ellipsoidal TM rectification and actual ellipsoidal UTM coordinates remain
unimplemented, as do the other unfinished E2 and later plan stages. This
increment supplies the exact spherical component and selection evidence.
