# Earth and sky development research

Initial research: 2026-10-18. This extends the
[Earth-first development plan](../DEVELOPMENT.md). All capabilities below are
planned. Production numerical values, including time, use `hyperreal::Real` or
types backed by it. External crates are development-only test/benchmark oracles
and algorithmic references; there is no external runtime backend in this design.

## Standards and reference implementations

Use an identified release of IERS conventions and an explicit amendment set.
The IERS distinguishes its published 2010 conventions from subsequently updated
working material; a moving web page is not a reproducible model identifier.
Record both algorithm and data versions. See
[IERS conventions](https://www.iers.org/iers/en/dataproducts/conventions/conventions)
and [updated material](https://iers-conventions.obspm.fr/conventions_material.php).

SOFA supplies standard astronomical algorithms and worked examples. Its
current software page identifies the 2023-10-11 release; use a pinned release
for fixtures. In particular, distinguish IAU 2006 precession from the compatible
IAU 2000A nutation model. See
[SOFA software](https://www.iausofa.org/current-software) and
[cookbooks](https://www.iausofa.org/cookbooks).

NAIF SPICE is the main reference for kernel semantics, frame/state transforms
and geometric versus aberration-corrected ephemeris queries. Rust examples and
API tradeoffs for SOFA-derived crates, ANISE, Lox, Satkit and Hifitime are in the
[ecosystem comparison](ecosystem.md). Their outputs can inform tests, but no
float result establishes a native exactness claim.

## Frame identity and coordinate kinds

An astronomical coordinate needs more than an `Eci` or `Equatorial` tag.
The proposed frame description carries orientation, origin, realization,
conventions and applicable epochs. Keep these distinctions in the public model:

| Concept | Required distinction |
| --- | --- |
| ICRS and ICRF | Reference system versus its identified realization; neither is a synonym for every frame named J2000. |
| BCRS and GCRS | Solar-system barycentric and geocentric spacetime coordinate systems. A full standards-based conversion has relativistic terms; an ordinary translated Cartesian model must identify its approximation. |
| ITRS and ITRF | Terrestrial reference system versus a particular realization, with coordinate epoch and station motion where applicable. |
| CIRS and TIRS | Intermediate celestial and terrestrial systems in the Earth-orientation chain. |
| J2000 and equinox-based frames | Distinguish a reference epoch, mean equator/equinox axes, a catalog convention and a backend's frame alias. |
| Equatorial, ecliptic and Galactic coordinates | Name the reference system/model and, when relevant, mean/true and fixed-epoch/of-date conventions. |
| TEME | A separately named convention for satellite workflows; never silently relabel SGP4 output as GCRS or ICRS. |
| Body-fixed frames | Body identity, orientation model/kernel, reference surface and planetocentric/planetographic convention. |

The foundational system distinctions and intermediate frames follow
[IERS Chapter 5](https://iers-conventions.obspm.fr/content/chapter5/icc5.pdf).
Backend aliases and body-frame conventions must be checked against
[NAIF frames](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/frames.html).
TEME is explicitly distinguished in
[Satkit's frame API](https://docs.rs/satkit/latest/satkit/frametransform/index.html).

Use different native concepts for a unit `Direction`, a finite `Position`, a
`State` containing position and velocity, a catalog source and an observation.
Directions rotate without origin translation; a finite source viewed from a
different origin needs distance/parallax information. A state also needs epoch
and the time variable with respect to which its velocity is defined.

These are proposed responsibilities, not fixed Rust signatures. Frame IDs may
be structural metadata, but angular/linear coordinates, rates and epochs are
`Real`-backed. Reuse Hyperlattice Cartesian carriers and centralize shared
linear algebra at the earliest sufficient native layer.

## A1 Time and immutable data contexts

Introduce native `Epoch<Scale>` and `Duration` roles backed by `Real`, with
typed scales UTC, TAI, TT, UT1 and TDB first, and TCG/TCB when their operations
are admitted. Preserve exact fractions at ingress. Do not adopt integer
nanoseconds, a Unix timestamp or a single binary64 Julian date as the core
carrier. Calendar components and data indices remain structural metadata.

SOFA uses two-part Julian dates to limit floating-point loss and a special UTC
encoding around leap seconds. UTC conversion needs leap information; UT1 needs
observational offsets. These conventions belong in explicitly named oracle
conversions, not implicit arithmetic on native epochs. See the
[SOFA time-scale cookbook](https://www.iausofa.org/s/sofa_ts_c.pdf).

Proposed native provider roles:

| Provider | Data and result contract |
| --- | --- |
| `LeapSecondProvider` | Pinned UTC/TAI history, supported date interval and explicit policy beyond known data; historical UTC behavior only within an admitted domain. |
| `EarthOrientationProvider` | UT1 offset, polar motion, celestial pole offsets and any needed rates; source status, units, uncertainty, coverage and interpolation model. |
| `EphemerisProvider` | Native state for named target, centre, frame and epoch; segment/model identity and validity. |
| `BodyOrientationProvider` | Native body orientation and required derivatives from an identified model/kernel. |

IERS Bulletin A includes estimates, predictions and errors for Earth orientation,
including polar motion, UT1-UTC and celestial pole offsets. Preserve their
status when constructing the context; predicted data must remain identifiable.
See [Bulletin A metadata](https://datacenter.iers.org/productMetadata.php?id=6).

Prepare immutable snapshots with content digests, source identifiers,
interpolation conventions and coverage. Native file readers supply `Real` values.
Keep data acquisition outside numerical evaluation; tests use pinned local data.
Do not silently substitute UTC for UT1, fill missing polar motion with zero,
extrapolate a table or select a different model. A deliberately simplified
model may authorize these assumptions and must retain that identity.

Differentiate coordinate epoch, frame/parameter reference epoch, catalog epoch,
observation epoch and emission epoch. Time-scale conversion and propagation
of a physical object to a different instant are different operations. Define
interpolation across UTC leaps on a suitable continuous scale, and expose the
validity and uncertainty of every time conversion model.

## A2 Earth orientation and celestial axes

The IERS terrestrial-to-celestial chain is
`r_GCRS = Q(t) R(t) W(t) r_ITRS`: polar motion, Earth rotation, and the celestial
pole/origin transformation. TIRS and CIRS expose its intermediate stages.
Use UT1 for Earth rotation and the specified dynamical time argument for the
precession/nutation model. CIO-based and equinox-based conventions require
consistent origins throughout. See
[IERS Chapter 5, sections 5.3–5.4](https://iers-conventions.obspm.fr/content/chapter5/icc5.pdf).

```mermaid
flowchart LR
    ITRS["ITRS / identified ITRF realization"] -->|"W: polar motion"| TIRS
    TIRS -->|"R: Earth rotation"| CIRS
    CIRS -->|"Q: celestial pole and origin"| GCRS
```

Admit a native model with explicit IAU/IERS conventions, TT/UT1 inputs, polar
motion and a stated celestial-pole-offset policy. Account for model completeness
separately from exact evaluation of its finite series. The low-level
[`sofars::pnp::c2t06a`](https://docs.rs/sofars/0.6.1/sofars/pnp/fn.c2t06a.html)
is a useful baseline oracle: it takes two-part TT and UT1 plus polar motion.
Observed celestial pole offsets require a compatible extended construction;
they are not arguments to that convenience routine.

Once this chain is admitted, add explicitly named equatorial/ecliptic and
Galactic orientation operations. Retain each adopted constant and model source.
Legacy equinox conventions and TEME can follow with separate fixtures. A single
sidereal rotation may be offered only as a named simplified model with its own
domain and accuracy qualification.

## A3 States and observers

For a time-dependent geometric transform using one common coordinate-time
variable, let `R` map source components to target components and `p` locate the
source origin in the target frame. Differentiating gives:

```text
r_target = R r_source + p
v_target = R v_source + dR/dt r_source + dp/dt
```

This product-rule identity defines the proposed native state-transform
contract. Rotating velocity alone omits motion of the frame. Changing the time
variable also needs the appropriate derivative conversion. A complete
relativistic transformation has a separate contract. NAIF similarly exposes
a [6×6 state transformation](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/cspice/sxform_c.html)
distinct from its position-only rotation.

Represent state-transform orientation derivatives and origin motion explicitly.
Generic algebra may belong in Hyperlattice; Hyperspatial owns frame/epoch/model
compatibility. Geometric composition must retain these terms, including inverse
and repeated transforms. Model validity and inherited evidence travel with the
state. Do not promise that light-time or refracted observations form the same
invertible transform group.

Start observers with a station fixed in a named terrestrial realization,
constructed from E1 geodetic/ECEF data. Its geodetic height is ellipsoidal;
orthometric heights require an admitted geoid conversion. The ENU basis uses
the reference ellipsoid normal. A gravity/plumb-line vertical is a separate
model. The observation epoch supplies the station's inertial state, including
velocity even when its terrestrial coordinates are held fixed.

Add moving observers through a native state provider. Later station models may
include reference-epoch velocity, plate motion, tides and loading; record which
displacements were evaluated. A requested model cannot quietly omit unavailable
terms. Observer identity includes its location/trajectory, epoch, frame and
local orientation. Atmospheric parameters are separate observation inputs.

## A4 Ephemerides and body orientation

Implement a native ephemeris evaluator over pinned data. An ephemeris query
names target and centre separately, observation time scale, output frame and
the requested geometric quantity. Keep body centres distinct from system
barycentres. Imported binary coefficients become exact represented `Real`
values; polynomial evaluation preserves that finite model, not an unsupported
claim of physical trajectory accuracy.

SPK is a candidate initial format. Type 2 stores position Chebyshev coefficients
and derives velocity; Type 3 stores separate position and velocity coefficients.
Their semantic difference matters for state acceptance. Specify supported types,
segment selection and kernel priority explicitly; SPK support is not a claim
of complete SPICE compatibility. See
[SPK required reading](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/spk.html).

Before admitting a native reader, cover bounded lengths/degrees, byte order,
nonfinite coefficients, malformed records, record endpoints, overlaps, gaps,
frame dependencies and deterministic load order. Keep data identity through
cache eviction and reload. Evaluation must respect work and memory budgets;
unsupported record types and missing orientation kernels fail explicitly.

Select fixtures by their actual kernel coverage. JPL's DE440 and DE441 have
different intended time spans and accuracy tradeoffs; a longer ephemeris is
not automatically the better modern-epoch reference. The coverage of a
particular distributed subset must be read from that asset. See
[JPL's DE440/DE441 description](https://ssd.jpl.nasa.gov/doc/de440_de441.html).

Add native body orientation separately from translational ephemerides, with
its own kernel/model and derivative requirements. Begin with synthetic data
whose polynomials and expected values are known exactly, then small pinned
real-kernel fixtures compared against ANISE and, where available, CSPICE in
development tooling. Large kernels need an explicit fixture acquisition step,
documented provenance and licensing; normal tests must not fetch them.

Ephemeris lookup and orbit propagation are separate capabilities. An initial
ephemeris evaluator does not imply a force model or numerical integrator.
Native SGP4/TLE handling may be a later project with TEME, epoch conventions
and model qualification retained; an external propagator remains dev-only.

## A5 Observed sky and correction policies

Provide two distinct source paths:

- **Catalog sources:** named catalog frame and epoch, direction, proper motion
  with its right-ascension convention, parallax and radial velocity where
  supplied. Preserve missing/uncertain distance; a nonpositive measured
  parallax must not become an invented reciprocal distance.
- **Solar-system sources:** native ephemeris states with target/observer origin,
  reception or transmission geometry and an explicit light-time policy.

Prepared astrometry contexts should reuse site, epoch and model calculations
across many targets. SOFA-derived `atco13` combines catalog and site data with
UTC, UT1 offset, polar motion, atmospheric inputs and wavelength; its many raw
arguments illustrate why the native API needs typed contexts. Its model also
includes a particular Earth ephemeris, which must match when comparing results.
See [the documented routine](https://docs.rs/crate/sofars/latest/target-redirect/x86_64-unknown-linux-gnu/sofars/astro/fn.atco13.html)
and [the crate's worked astrometry example](https://docs.rs/sofars/0.6.1/sofars/).

Return named stages such as geometric line of sight, astrometric direction,
vacuum apparent direction and refracted observed direction. Retain a correction
record identifying light-time, aberration, gravitational deflection, observer
displacement, atmosphere and convention choices actually applied. Stage
composition must reject applying a correction twice. Do not expose a single
ambiguous `apparent: bool` switch.

For ephemeris queries, SPICE distinguishes geometric, one-iteration and
converged Newtonian light-time corrections, optional stellar aberration, and
reception versus transmission. Its corrected-state routine does not apply
relativistic light propagation. Noninertial output-frame epochs also depend on
the correction convention. See
[`spkezr_c`](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/cspice/spkezr_c.html).
The native policy must state its equation, iteration budget, residual evidence
and model limitations. Coverage checks include shifted emission epochs.

Use explicit azimuth origin/sense, altitude versus zenith distance, and angular
wrapping. Zenith/nadir azimuth and a zero-range direction require defined
failure or ambiguity outcomes. Refraction needs a named model, atmospheric
inputs, wavelength and validity domain; an out-of-domain horizon calculation
must not silently extrapolate. An inverse observation operation requires its
own existence, branch and conditioning contract.

Sky-to-image WCS, instrument/mount models, event finding and orbit integration
are later extensions. They should consume these native qualified results,
without expanding the initial Earth/local-frame deliverable.

## Missing acceptance fixtures and development order

All rows are future work. The scaffold has no numerical transformation tests.

| Stage | Required evidence before admission |
| --- | --- |
| A1 time/data | Exact sub-nanosecond fractions; same instant across scales; leap boundaries and historical-domain limits; missing/outdated data; observed/predicted status; interpolation edges; cache replay; exact oracle-conversion loss. |
| A2 Earth/celestial rotation | Published SOFA/IERS examples with pinned model/data; basis orientation, handedness, orthogonality and inverse identities; TT/UT1 confusion regression; polar motion and celestial pole corrections independently varied; explicit simplified-model qualification. |
| A3 state/observer | Exact synthetic rotating/translating frames; analytical velocity expectations; a stationary terrestrial site has nonzero inertial velocity; repeated composition/inversion; origin, epoch and time-derivative mismatches; ellipsoid-normal versus radial Up. |
| A4 ephemerides | Exact synthetic Chebyshev cases; Type 2 derivative versus Type 3 stored velocity; malformed/oversized files; record and coverage boundaries; overlapping segment priority; centre chains; identical pinned queries against development oracles. |
| A5 observations | Catalog proper-motion conventions; missing distance; finite-distance parallax; reception/transmission and light-time limits; shifted epochs beyond coverage; correction duplication; zenith/poles/zero range; refraction domain; separate geometric/vacuum/refracted results. |

The first complete astronomy workflow should transform a terrestrial station
and native Sun/Moon ephemeris state at a pinned epoch into a geometric local
direction, then add explicitly requested observational corrections. Retain the
frame, time, dataset, model and uncertainty records through every stage.

Cross-checks need the same definition of the observable. JPL Horizons documents
ICRF and several of-date/ecliptic conventions using IAU76/80; a default Horizons
apparent coordinate is not automatically the same quantity as an IAU2006/2000A
SOFA result. Pin centre, time scale, axes, corrections, ephemeris and output
units before interpreting a residual. See the
[Horizons manual](https://ssd.jpl.nasa.gov/horizons/manual.html).

Use algebraic identities, independent derivations and certified enclosures in
addition to oracle agreement. Track arithmetic evidence, finite-model error,
data uncertainty and observational-model limits separately. Test unresolved
predicates, cancellation and resource exhaustion without turning them into
zero coordinates or partial success. Add development dependencies only with a
concrete test or benchmark; keep production numerical paths entirely native.
