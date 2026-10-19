# Hyperspatial development plan

## Scope and status

Initial plan: 2026-10-18. Source baseline:
`dbe99e3cc145e8ca58a53ade6e79fd46311972fb`. Development branch: `hyperspatial`.

Build a native coordinate transformation layer for the Hyper stack, beginning
with Earth geodesy, map projections, and local frames. Extend the same evidence
and reference-frame model to Earth orientation, celestial coordinates,
observers, and ephemerides. Implementation is incremental; the full plan remains
active. E1 provides units, validated ellipsoids, forward and closest-point
inverse geodetic/ECEF operations, atomic batches and anchored ENU/NED transforms
with retained evidence. E2 provides spherical/pseudo-Mercator and ellipsoidal Mercator with explicit
longitude policies; its remaining projection/geodesic work, E3–E4 and A1–A5
remain planned. Further testing/admission work is open.
The native, refinable inverse-isometric-latitude scalar in Hyperreal supplies
the ellipsoidal projection inverse; its
[scalar admission](../hyperreal/docs/admission-inverse-isometric-latitude.md)
and [projection admission](docs/admission/e2-ellipsoidal.md) distinguish the
scalar proof from coordinate identity, range evidence and downstream composition.
E2 also provides the exact spherical transverse chart and UTM angular zone
selection, with retained inverse relations and explicit pole conventions. The
[transverse admission](docs/admission/e2-transverse.md) records this prerequisite.
Ellipsoidal Transverse Mercator/UTM and geodesic measurements remain open within E2.
Native Carlson RF/RD elliptic integrals are an additional scalar prerequisite
for exact rectification, with [scalar admission](../hyperreal/docs/admission-elliptic-integrals.md).
Native Legendre F/E, their continuous inverses and shared real Jacobi sn/cn/dn
now supply further scalar prerequisites. The [Jacobi admission](../hyperreal/docs/admission-jacobi.md)
records retained square-sum relations and positive dn. Hyperlattice now provides
complex Jacobi continuation for admitted real m < 1, with explicit pole errors
and exact relations reusable by strict spatial consumers. Its
[complex admission](../hyperlattice/docs/admission-complex-jacobi.md) and
[scalar relation admission](../hyperreal/docs/admission-jacobi-rational-relations.md)
record the construction and proof budgets. The
[verification record](verification/2026-10-19-e2-complex-jacobi/README.md)
records passing stable/MSRV checks, all four cache configurations, eight
independent coverage gates, directed selection and the completed ASan campaign.
Thompson/Lee rectification and a complete ellipsoidal transverse
projection remain open.
See the [spherical Mercator admission](docs/admission/e2-mercator.md),
the [forward/local admission](docs/admission/e1-native.md)
and [closest-point inverse admission](docs/admission/e1-inverse.md).
Dedicated stable/MSRV CI, native WASM compilation, production coverage,
source-consumer checks and workspace adversarial/benchmark integration are
recorded in the [testing admission](docs/admission/testing.md).
The [directed-testing extension](docs/admission/e1-directed-testing.md) adds
the shared 22-class scalar inventory, 484 ordered pairs, cache-layout checks,
per-family lexicase selection, native dispatch observations and ASan campaigns.
These extend E1 evidence; they do not close infinite-domain coverage or prove
native scalar work/heap bounds.

Design constraints confirmed in this planning session: **production numerical
scalars are `hyperreal::Real` or types backed by it**, and **external crates are
development-only test/benchmark oracles and algorithmic inspiration**. This
includes time, units, model parameters, coefficients, positions and velocities.
Runtime dependencies belong to the native Hyper stack. Integer identifiers,
indices and resource counters are bookkeeping, not alternative numeric carriers.

The governing order remains
**exactness > completeness > performance > memory > binary size > source size**,
within each admitted domain and resource contract.
The workspace [properties](../../properties.md) and
[admission protocol](../../docs/admission.md) govern implementation and release.

## Existing layers and ownership

| Layer | Existing capability | Hyperspatial relationship |
| --- | --- | --- |
| [Hyperreal](../hyperreal/README.md) | Exact rational and decimal input, symbolic constants, computable reals, trig, `atan2`, `hypot3`, stable arithmetic, typed scalar failure | Reuse scalars and reducers. Add generally useful scalar identities here, with scalar acceptance evidence. |
| [Hyperlattice](../hyperlattice/README.md) | `Point3`, `Vector3`, `Matrix3`, `Matrix4`, affine/projective transforms, checked inverses, batch operations | Reuse Cartesian carriers and linear algebra. Generic rotation/state algebra belongs at the earliest sufficient layer. |
| [Hyperlimit](../hyperlimit/README.md) | Explicit policies, certified decisions, unresolved outcomes, evidence | Reuse domain and branch decisions; do not introduce geodetic epsilon predicates. |
| Hyperspatial | Native units, ellipsoids, forward/inverse geodetic and local Cartesian operations | Own coordinate semantics, frame realizations, reference epochs, coordinate operations, model selection, and data-provider contracts. |
| Hypercurve, Hypermesh, CSGRS | Native curves, regions, meshes, topology and modeling composition | Consume explicit Cartesian results. Own geometry validity and adapters for nonlinear images; retain original sources and transformation evidence. |
| Development oracles | PROJ, GeoRust, astronomy/time libraries | Test/benchmark comparisons and algorithm research only; adapters remain in development tooling. |

`geo` is already a Hypercurve development dependency; `geographiclib-rs` is in
the workspace lockfile through it. This does not expose native geodetic APIs.
The existing point/matrix types do not carry geodetic units, datums, or epochs.
Native dependencies are Hyperreal, Hyperlattice and Hyperlimit. External test
support is development-only; add dependencies with their first admitted consumer.
The [ecosystem comparison](docs/ecosystem.md) records
candidate references and development oracles. Do not expose external crate
types or backend-selection features in the production API.

## Evidence and numerical meaning

Every result must retain separate claims about:

1. **Numerical certainty:** what the arithmetic, predicates, and refinement
   actually establish for the represented inputs and equations.
2. **Model fidelity:** the ideal operation or finite model being evaluated,
   approximation bounds where established, validity area and epoch interval.
3. **Input qualification:** measurement uncertainty, covariance or bounds when
   supplied, and the assumptions used to propagate them. Missing uncertainty
   information means unknown accuracy, not zero uncertainty.
4. **Provenance:** frame and datum realization, coordinate/reference epochs,
   operation identity, conventions, implementation version, and dataset digests.
5. **Topology and geometric fidelity:** separate claims for transformed curves,
   regions, meshes, sampled bounds, or exported geometry where applicable.

Exact evaluation of a finite projection series, ephemeris polynomial, or grid
interpolant is exact for that represented model. It does not certify agreement
with an ideal projection, the continuous trajectory, or the physical Earth.
Keep series truncation, interpolation, measurement, and float-boundary errors
separate. Use independently justified bounds to make stronger claims.

`GeometryCertainty::Approximate512Consumed` describes a particular predicate
terminal. It must not label arbitrary external binary64 calculations or model
uncertainty. Importing an external float fixture or file coefficient into `Real`
preserves that represented binary value and must retain its inherited
qualification. Native file readers should decode represented values directly
into `Real`; no transformation stage computes through a float or integer scalar.

## Proposed API responsibilities

These names describe design roles, not committed public signatures.

| Concept | Required semantics |
| --- | --- |
| `Latitude`, `Longitude`, angular and length quantities | `Real`-backed values, named unit constructors, exact text ingress, latitude validation, explicit wrapping policy; distinguish a normalized longitude from an unrolled path parameter. |
| `Ellipsoid` | Exact defining parameters with validation and explicit sphere/oblate/prolate support. An ellipsoid alone is not a datum or frame realization. |
| `GeodeticPosition`, `EcefPosition`, `ProjectedPosition` | Distinct coordinate kinds, units, frame realization and applicable epoch. Cartesian storage reuses Hyperlattice. |
| `LocalFrame`, `LocalPosition`, `LocalVector` | Origin, ENU/NED convention, surface normal, reference frame, and local-frame identity. Mixing different origins fails or requires an explicit conversion. |
| `CoordinateOperation` | A named operation, direction, source/target semantics, admitted domain, and evidence-preserving composition. Inverse construction is fallible. |
| `TransformContext` | Explicit predicate policy, precision/work/memory/cancellation budgets, immutable data snapshot, and permitted approximation/fallback policy. |
| `TransformOutcome<T>` | Native value plus reusable evidence, inherited qualification, model identity, and provenance; subsequent operations consume the qualification too. |
| Time, state, observer and ephemeris concepts | Defined in [Earth and sky research](docs/astronomy.md); time-dependent and observational operations have separate contracts. |

Coordinate kind can use static type distinctions while arbitrary CRS/frame
definitions use runtime identities. A source and target sharing the same
coordinate kind does not establish that their frames, origins, epochs, units,
or local anchors agree. Validate these at composition boundaries.

Separate preparation from evaluation so ellipsoid constants, local rotations,
operation selection and data resolution can be reused. Preserve outputs under
all documented cold/warm/evicted/concurrent cache states. Cache identity must
include model, data, epoch/interpolation and policy where relevant.

Use typed failures for invalid input, undefined or ambiguous inverse,
unsupported operation/model, incompatible frame, missing epoch/data,
out-of-coverage query, undecided predicate, and resource exhaustion. No failure
becomes NaN, a zero coordinate, an unchanged input, or a complete partial batch.
Define atomic batch behavior first; any per-element/provisional mode is explicit.

## Earth development stages

### E1 Geodetic and local frames

Start with named units, validated ellipsoids, geodetic to ECEF conversion,
ECEF to geodetic inversion, and anchored ENU/NED transforms. Use native
Hyperlattice rotations/translations for ECEF to local Cartesian operations.
Distinguish a local Cartesian displacement from a surface geodesic distance.

The first inverse implementation must publish its supported spatial domain,
including ordinary below-ellipsoid heights. Define behavior at poles, at the
Earth's centre, and where deep-interior geodetic inverses are nonunique.
An arbitrary longitude convention at a pole must remain visible. Do not derive
ellipsoidal Up by normalizing the geocentric radius.

### E2 Projections and geodesic measurements

Implement spherical Mercator/Web Mercator with an explicit singularity and
longitude policy, then ellipsoidal Mercator and Transverse Mercator/UTM.
Record each projection's parameters, area of use, inverse domain and model.
Handle zone/hemisphere selection, antimeridian seams, and Norway/Svalbard rules
explicitly. Web display clipping is a separate boundary operation.

Implemented prerequisites: exact spherical transverse forward/inverse on its
closed longitude chart, scaled inverse for huge finite eastings, and exact UTM
zone/hemisphere selection with special areas. Ellipsoidal conformal-latitude
conversion can reuse the native isometric scalar. Rectification still needs
its own finite-model or certified-remainder contract; the spherical chart and
zone selector do not supply ellipsoidal projected coordinates.

The exact rectification path now has native Carlson RF/RD and complete/incomplete
Legendre F/E integrals (real parameter m < 1, unwrapped real amplitudes). The
[Legendre admission](../hyperreal/docs/admission-legendre-integrals.md) records
period reduction, positive second-kind formulas, and independent development
oracles. Native `jacobi_amplitude` and `inverse_elliptic_e` now retain unwrapped
inverse relations and use certified rational root enclosures; their
[admission](../hyperreal/docs/admission-inverse-legendre.md) distinguishes the
retained exact value from finite evaluation bounds. Native `jacobi_sncndn`
now shares that amplitude, reduces its periods with a certified quotient,
and retains unit/elliptic square relations and the positive dn branch. Its
[admission](../hyperreal/docs/admission-jacobi.md) records the real parameter
domain and bounded relation recognition; [verification](verification/2026-10-19-e2-jacobi/README.md)
records executed evidence and limits. Hyperlattice's `Complex::jacobi_sncndn`
continues these functions using complementary and reciprocal modulus
transformations, including negative parameters and finite quarter-period
points. An independent MPFR Taylor-ODE oracle supplies development comparisons;
native scalar relation proofs let ordinary complex arithmetic preserve unit
and elliptic norms. The [spatial testing contract](docs/admission/e2-complex-jacobi.md)
requires those norms to remain consumable as geodetic inputs. Complex
parameters and m=1 are outside this increment. Next, admit the Thompson/Lee
mapping and its certified inverse. Its rectification uses Jacobi's epsilon
function, whose argument differs from the amplitude accepted by incomplete
Legendre E. The real axis can compose E with the retained Jacobi amplitude;
complex continuation must preserve quasi-periods, finite quarter-period
values and pole diagnostics. The two intermediate-map inversions need
certified enclosures and a named chart or sheet; convergence of ordinary
complex Newton iteration alone is insufficient. See
[Jacobi epsilon and its addition law](https://dlmf.nist.gov/22.16.ii).
Other references are
[DLMF integral relations](https://dlmf.nist.gov/19.25),
[Jacobi computation](https://dlmf.nist.gov/22.20), and
[Karney's exact and series formulations](https://arxiv.org/abs/1002.1417).
The native scalar prerequisite does not supply those remaining algorithms.
If a finite Krueger model is also admitted, its forward and inverse contracts
must be explicit: independently truncated forward/inverse series are not exact
inverses of one finite forward map.

Admit Transverse Mercator series only with a stated finite-model contract or
certified remainder bounds relative to the ideal projection. Exact scalar
evaluation does not remove truncation error. GeographicLib supplies useful
geodesic references for distances, bearings, destinations and polygon area;
near-antipodal cases and multiple shortest geodesics need explicit semantics.
See [PROJ Transverse Mercator](https://proj.org/en/stable/operations/projections/tmerc.html)
and [GeographicLib Rust](https://docs.rs/geographiclib-rs/0.2.7/geographiclib_rs/).

### E3 Datums and external coordinate systems

Add explicit datum/frame realization and operation identifiers, static Helmert
transforms, and later rates with separate coordinate and parameter reference
epochs. Preserve the specified position-vector or coordinate-frame rotation
convention and distinguish linearized from full rotation models. Frame changes
at one epoch and propagation of a moving point to another epoch are different
operations. See [PROJ Helmert](https://proj.org/en/stable/operations/transformations/helmert.html).

Provide deterministic CRS lookup and prepared operation selection before
supporting grid shifts, geoid heights and deformation models. Pin registry and
grid versions, interpolation rules, coverage and checksums. Require intentional
permission for lower-accuracy fallback; expose the operation actually consumed.
Ellipsoidal, orthometric and other gravity-related heights remain distinct.

### E4 Geometry integration

Add native format boundaries and development-only GeoRust/PROJ comparisons with
explicit conversion loss. Existing native geometry remains authoritative.
Nonlinear projection of a line or arc generally produces a different curve
family; mapping vertices and reconnecting them does not establish the exact
image. Admit retained parametric
images or a separately certified approximation and topology contract.
Bounding-box corner transforms alone do not enclose general nonlinear images.

## Astronomy development order

After E1 establishes frame and observer-location foundations, develop time and
data contracts, Earth orientation, celestial states, ephemeris queries, then
observational corrections. These stages may overlap later Earth work without
making astronomy model/data modules mandatory for E1/E2. All production stages
use native implementations and `Real`-backed quantities. Detailed requirements,
sources and comparison APIs are in [Earth and sky research](docs/astronomy.md).

## Acceptance before implementation

For each patch, identify the owning layer, concrete domain, relevant property,
missing/failing regression and downstream consumers before modifying code.
The following matrix governs the complete stages. The E1 admission record names
implemented tests; an entry in this matrix alone is not acceptance evidence.

| Stage and owner | Property focus | Required acceptance cases |
| --- | --- | --- |
| E1 units and ellipsoids, Hyperspatial over Hyperreal | P1–P7, P26 | Exact decimal/angle ingress; invalid ellipsoids and latitude; explicit longitude wrap; nonfinite file/oracle ingress rejection. |
| E1 forward/inverse and local frames, Hyperspatial over Hyperlattice/Hyperlimit | P8–P11, P17–P20 | Exact equator/pole/sphere identities; origin maps to zero; basis orientation; below-surface and large heights; centre/inverse ambiguity; frame mismatch; repeated inverse/composition with evidence retained. |
| E2 projections and geodesics, Hyperspatial | P6, P9, P17–P20 | Poles and seams; zone boundaries; independently derived special cases; approximation enclosures; coincident/near-antipodal points; multiple branches. |
| E3 registry and native providers, Hyperspatial | P4–P6, P19, P26–P27 | Missing data; coverage edges; explicit fallback; height and epoch mismatch; pinned snapshot replay; cache replacement/eviction; exact unit conversions. |
| E4 geometry and boundaries, existing geometry owners | P15–P20, P25–P30 | Nonlinear edges and extrema; holes and seam splitting; provenance through export/import; real C/Python/JS consumers as introduced. |
| Astronomy A1–A5, Hyperspatial/providers | P1–P11, P17–P20, P26–P30 | The time, state, observer and kernel matrix in the astronomy research. |

Round trips alone can hide paired implementation mistakes. Combine exact
identities and certified enclosures with independent published examples and
pinned external comparisons. Two ports of the same algorithm are not independent
mathematical proofs. Record numerical/model tolerances separately from topology
decisions; do not use comparison tolerances as native equality policies.

Exercise every admitted scalar representation, strict and qualified paths,
tiny work/memory budgets, cancellation and recovery. Retain minimized fuzz
failures alongside full cases. Before performance claims, measure the same
completed contract including provenance, output materialization and failures.

## Distribution and open decisions

Before a release, validate MSRV, features, documentation, packaging and clean
external consumers through construct/transform/compose/evidence/export/import.
Dedicated CI now checks stable and Rust 1.95, production coverage and a native
WASM library build. A WASM runtime consumer and packaging remain open; the
separate README consumer validates local source use. Every push
still requires the full repository pre-push validation set.

The initial design keeps these decisions open until a concrete acceptance
experiment resolves them:

- Broader dispatch, sanitizer and resource evidence for the implemented exact
  closest-point inverse and its admitted ellipsoid domain.
- Projection remainder certificates and general geodesic representations.
- Static/runtime frame type balance and serialization of qualified outcomes.
- `Real`-backed time API and exact ingress/egress without hidden nanosecond or
  binary64 loss.
- Native ephemeris evaluation algorithms and the first supported SPK types.
- Development-oracle versions, coverage, fixture reproducibility and source/data
  license requirements; no external runtime backend is planned.
- Shared state-transform algebra placement and nonlinear geometry integration.

Scaffold validation is recorded in the
[initial admission record](docs/admission/2026-10-18-scaffold.md). It establishes
package integration only; all transformation acceptance remains open.
