# Coordinate transformation ecosystem

Research snapshot: 2026-10-18. This records the initial planning discussion,
with **Earth geodesy, map projections, and local frames first**. Linked versions
identify documentation inspected, not dependency selections or a promise that
each is the newest release. API sketches are descriptive; no external backend
was installed, integrated, benchmarked, or validated by this scaffold.

The planning decision is firm: these external crates are **development-only
test/benchmark oracles and algorithmic inspiration**. Production algorithms and
all numerical domain quantities use `hyperreal::Real` or `Real`-backed types.
External libraries are not runtime dependencies or public API carriers.

## What the workspace already supplies

| Existing API | Reuse in Hyperspatial | Missing domain semantics |
| --- | --- | --- |
| Hyperreal `Real`, exact text/rational construction, `sin`, `cos`, `atan2`, `hypot3`, `to_radians` | Native scalars, symbolic angles and stable elementary operations | Units, ellipsoids, datums, projections, astronomical models and measurement uncertainty |
| Hyperlattice `Point3`, `Vector3`, `Matrix3`, `Matrix4` | Cartesian storage and shared linear algebra | Frame identity, reference epoch, coordinate kind and observer origin |
| `Matrix4::affine_translation`, `rotation_x/y/z`, `transform_point3`, `transform_direction3` and batches | Prepared local rotations/translations; distinguish points from free vectors | Nonlinear coordinate operations; time derivatives needed for moving-frame states |
| Hyperlimit `PredicatePolicy`, `PredicateOutcome::Decided` / `Unknown` | Explicit domain/branch decisions with certainty and resource limits | Geodetic validity and operation-selection contracts |
| CSGRS context and solid transforms; Hypercurve native regions | Downstream construction, composition and evidence retention | Exact nonlinear images of curves/surfaces and CRS-aware topology |

Sources: [Hyperreal](../../hyperreal/README.md),
[scalar functions](../../hyperreal/src/real/arithmetic/elementary_functions.rs),
[linear algebra primitives](../../hyperreal/src/real/arithmetic/linear_algebra.rs),
[Hyperlattice transforms](../../hyperlattice/src/matrix/core.rs),
[predicates](../../hyperlimit/src/predicate.rs),
[CSGRS certainty](../../../src/context.rs),
[solid transforms](../../../src/solid.rs), and
[Hypercurve](../../hypercurve/README.md).

The workspace's `geo` 0.33.1 use is a
[Hypercurve development dependency](../../hypercurve/Cargo.toml).
`geographiclib-rs` 0.2.7 is already transitively present in
[Cargo.lock](../../../Cargo.lock). Neither provides a native exact transformation
layer to the workspace. Generic float traits in external crates should not be
assumed to accept `Real` or preserve its meaning.

## Earth crates and their APIs

| Crate and inspected docs | API shape and coverage | Implication for this workspace |
| --- | --- | --- |
| [`proj` 0.31.0](https://docs.rs/proj/0.31.0/proj/) | `Proj::new_known_crs(from, to, area)`, `convert`, `convert_array`, `ProjBuilder`; bindings to native PROJ 9.6.x, CRS database and grids | Broad external comparison tool. The ordinary `Coord` surface is 2D; do not infer access to every native PROJ 3D/4D operation. Native libraries/data affect the development harness. |
| [`geodesy` 0.15.0](https://docs.rs/geodesy/latest/geodesy/) | `Minimal::new()`, `ctx.op("utm zone=33")`, `ctx.apply(op, Fwd, &mut coordinates)`; pure Rust operation pipelines, projections and datum operations | Useful preparation/context design and native algorithm reference. Its documentation presents an experimental platform, with a smaller operational scope than PROJ. |
| [`proj4rs` 0.2.1](https://docs.rs/proj4rs/latest/proj4rs/) | `Proj::from_proj_string`, `transform(&from, &to, &mut xyz)`; pure Rust, PROJ-style definitions and WASM support | Lightweight comparison candidate. Geographic arguments use radians. Record coverage, grid behavior and WASM error settings in each comparison. |
| [`proj-core` 0.11.0](https://docs.rs/proj-core/0.11.0/proj_core/) | `Transform::new`, `convert`, `convert_3d`, inverse/batches; CRS lookup, grid providers and operation selection | Pure Rust algorithm/API reference with explicit selection diagnostics. Audit its supported CRS/operation subset before using it as an oracle. |
| [`geographiclib-rs` 0.2.7](https://docs.rs/geographiclib-rs/0.2.7/geographiclib_rs/) | `Geodesic::wgs84()` with direct/inverse traits, `GeodesicLine`, `PolygonArea`; ellipsoidal geodesics using binary64 | Strong reference for surface distance, bearings, destinations and area. It is not a general CRS pipeline or an exact-scalar implementation. |
| [`jord` 0.19.0](https://docs.rs/jord/latest/jord/) | `GeodeticPosition`, `NVector`, `Angle`, `Length`, `Ellipsoid`, `NedFrame::from_geodetic`; spherical navigation and local/ellipsoidal coordinate operations | Good reference for named units and local-frame ergonomics. Its mathematical/model use of “exact” is not the workspace's arithmetic certification contract. |
| [`nav-types` 0.5.2](https://docs.rs/nav-types/latest/nav_types/) | `WGS84`, `ECEF`, `ENU`, `NED`, `NVector`; `from_degrees_and_meters` and checked constructors; nalgebra-based | Compact typed-coordinate example. Numeric trait bounds do not make it a drop-in `Real` backend. Its spherical distance helper is not an ellipsoidal geodesic. |
| [`geoconvert` 1.0.2](https://docs.rs/geoconvert/latest/geoconvert/) | `LatLon::create`, `to_utmups`, `to_mgrs(precision)` | Focused WGS84 grid-reference conversion. MGRS resolution/quantization must be represented explicitly on import/export. |
| [`map_3d` 0.1.7](https://docs.rs/map_3d/latest/map_3d/) | Free functions such as `geodetic2ecef(lat, lon, alt, ellipsoid)` returning tuples; radians and metres | Simple ECEF/local-frame reference. Lightweight sidereal ECI conversions are a different fidelity contract from an IERS Earth-orientation model. |
| [`geo3d`](https://docs.rs/geo3d/latest/geo3d/) | `Geodetic`, `Ecef`, `Enu`, `Ned`, `Aer`, reusable `LocalFrame`, look angles | Useful observer/local-coordinate API example. Audit the named celestial model rather than treating an `Eci` label as a complete frame definition. |
| [`geo` 0.33.1](https://docs.rs/geo/0.33.1/geo/) | `Point`, `Coord`, `MapCoords`, `AffineOps`, metric-space `Distance`/`Bearing`/`Destination`, optional PROJ support | Development comparisons; keep Hypercurve/Hypermesh as native geometry owners. Nonlinear vertex mapping alone does not prove curve or topology fidelity. |

### Conventions that development harnesses must preserve

`proj::new_known_crs` normalizes geographic coordinate ordering to longitude,
latitude in degrees, while lower-level projection/pipeline interfaces have
their own unit conventions. `proj4rs` uses geographic radians. Geodesy's
`Coor2D::geo` constructor accepts latitude then longitude in degrees and adapts
to its internal coordinate convention. A tuple is therefore insufficient to
establish units or axis order. Use named native quantities and test each oracle boundary
against asymmetric coordinates. See the
[PROJ wrapper API](https://docs.rs/proj/latest/proj/struct.Proj.html),
[Proj4rs documentation](https://docs.rs/proj4rs/latest/proj4rs/), and
[Geodesy example](https://docs.rs/geodesy/latest/geodesy/).

Operation success also needs interpretation. `proj-core` exposes
`convert_with_diagnostics`, `convert_3d_with_diagnostics`, selected-operation
metadata and fallback diagnostics; record this information with comparison
results. Its sampled bounds API must not be promoted to a native
proof of nonlinear extrema. See
[`Transform`](https://docs.rs/proj-core/latest/proj_core/transform/struct.Transform.html).

Native constructors should be fallible on invalid latitude, ellipsoid or frame
parameters. Development harnesses must distinguish external panic, nonfinite,
partial-batch, missing-grid and fallback behavior from completed comparisons.
Native implementations must provide the explicit outcomes in the
[development plan](../DEVELOPMENT.md). This is an integration requirement, not
a claim that all listed libraries behave alike.

## Astronomy crates and API comparison

| Crate and inspected docs | API shape and coverage | Proposed role / limitation |
| --- | --- | --- |
| [`sofars` 0.6.1](https://docs.rs/sofars/0.6.1/sofars/) | Pure Rust SOFA-derived routines; `pnp::c2t06a` builds a celestial-to-terrestrial matrix from TT, UT1 and polar motion; `astro::atco13` maps catalog data to observed coordinates; reusable `IauAstrom` | Algorithm and fixture reference. Native APIs need typed `Real`-backed quantities and model/data qualification. Its documented SOFA terms accompany the MIT license. |
| [`erfa` 0.2.1](https://docs.rs/erfa/0.2.1/erfa/) | Pure Rust ERFA-derived time, precession/nutation and coordinate routines | Alternative low-level reference; verify routine coverage rather than assuming the complete C ERFA API is present. |
| [`anise` 0.10.6](https://docs.rs/anise/0.10.6/anise/) | `Almanac`, `Orbit`, `transform_to(state, frame, aberration)`, `rotate`, `translate`; kernel-backed ephemerides/orientations, Hifitime epochs | Candidate SPICE-compatible oracle and state/frame design reference. Pin supported kernel types, load order and coverage; record binary64 comparison limits and units. |
| [`lox-space` alpha documentation](https://docs.rs/lox-space/latest/lox_space/prelude/enum.Frame.html) | Static frame types plus runtime `Frame`; `TryRotation::try_rotation(origin, target, Time<T>)`; rotation/time providers | Useful model for static and dynamic frame interoperability. Alpha APIs are provisional. Default provider assumptions must be exposed. |
| [`satkit` 0.20.2](https://docs.rs/satkit/latest/satkit/frametransform/index.html) | Quaternion frame rotations, separate full/approximate Earth models, position-and-velocity transforms and TEME support | Satellite-oriented reference. Distinct APIs for orientation and state transport are useful; advertised model accuracy is not native arithmetic proof. |
| [`hifitime` 4.3.1](https://docs.rs/hifitime/4.3.1/hifitime/) | `Epoch`, `Duration`, named time scales, integer nanosecond representation | Time API reference and development oracle. Native epochs/durations use `Real`-backed quantities; oracle resolution must not constrain them. |

The Lox
[default rotation provider](https://docs.rs/lox-space/latest/lox_space/frames/providers/struct.DefaultRotationProvider.html)
documents zero polar motion and zero corrections without EOP data. That is an
explicit model choice Hyperspatial must record, not infer from a successful
rotation. Similar scrutiny is needed for all convenient default contexts.

## Initial decision and follow-up experiments

Build production algorithms on Hyperreal, Hyperlattice and Hyperlimit throughout
the Earth and astronomy stages. Use GeographicLib and PROJ as proposed
development oracles, with pure Rust alternatives for additional comparisons.
Use SOFA/IERS definitions for native astronomy algorithms and evaluate
ANISE/SPICE as ephemeris oracles. External runtime adapters are outside the
agreed design. Native data-provider interfaces will supply `Real`-backed values.

Before selecting a development oracle, run a small, pinned experiment that records:

- Concrete operations and domains, including inverse ambiguity and singularities.
- Axis order, angular/linear/time units, dimensions, datum and frame realizations.
- Error, batch, data-coverage, fallback and retained-provenance behavior.
- Supported platforms, native build dependencies, feature combinations and MSRV.
- Exact-to-float/time conversion loss in the harness and comparison limits.
- License and data-distribution requirements for the selected versions and assets.

External baseline timings may compare different numerical contracts, so report
that difference explicitly. Native performance claims require identical
completed contracts. A benchmark that drops context preparation, evidence,
data loading or failure handling cannot establish the cost of that contract.

The extended [Earth and sky plan](astronomy.md) defines the astronomy stages,
observer semantics, ephemeris contracts and missing acceptance fixtures.
