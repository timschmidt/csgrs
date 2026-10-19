# hyperspatial

`hyperspatial` is the native geospatial and astronomical coordinate
transformation layer of the Hyper geometry workspace. Its first development
target is Earth geodesy, map projections, and local reference frames. Later
work connects terrestrial positions to celestial frames, observers, and
ephemerides.

The first implementation provides exact angular/length units, validated
spherical/oblate/prolate ellipsoids, geodetic/ECEF conversion, and anchored
ENU/NED position/vector transforms, plus spherical/pseudo-Mercator projections.
Reference identity, coordinate epoch,
source construction and predicate evidence survive supported compositions.
The package is unpublished; the development plan is still in progress.

Native implementations will reuse `hyperreal`, `hyperlattice`, and `hyperlimit`.
All production numerical quantities, including coordinate epochs, use
`hyperreal::Real` or types backed by it. External crates are development-only
test/benchmark oracles and algorithmic inspiration.
Results must preserve exact input meaning and distinguish numerical certainty
from model approximation, measurement uncertainty, and dataset provenance.

```rust
use std::sync::Arc;
use hyperspatial::*;

let mut context = Context::new(PredicatePolicy::STRICT, Limits::default());
let surface = Arc::new(Ellipsoid::wgs84(&mut context)?);
let origin = GeodeticPosition::new(
    Latitude::new(Angle::degrees(Real::zero()), &mut context)?,
    Longitude::unrolled(Angle::degrees(Real::zero())),
    Length::metres(Real::zero()),
    surface,
    ReferenceFrame::fixed("example:static-earth")?,
    InputProvenance::new("mathematical example", InputAccuracy::ExactModel),
    &mut context,
)?;
let ecef = origin.to_ecef(&mut context)?;
assert_eq!(ecef.coordinates().x, Real::from(6_378_137));
let local = LocalFrame::new(&origin, LocalAxes::Enu, &mut context)?;
let zero = local.to_local_position(&ecef, &mut context)?;
assert_eq!(zero.coordinates().x, Real::zero());
assert_eq!(zero.receipt().certainty(), Certainty::Exact);
let inverse = ecef.to_geodetic_closest(
    origin.ellipsoid().clone(), PoleMeridian::Reject, &mut context,
)?;
assert_eq!(inverse.height().as_metres(), &Real::zero());
assert!(matches!(inverse.source(), GeodeticSource::EcefClosest { .. }));
let projection = SphericalMercator::web_mercator(
    origin.reference().clone(), LongitudePolicy::WrapSigned, &mut context,
)?;
let projected = projection.project(&origin, &mut context)?;
let geographic = projection.unproject(&projected, &mut context)?;
assert_eq!(geographic.height().as_metres(), &Real::zero());

// The scalar prerequisite for ellipsoidal Mercator is also available.
// For b/a=3/5 and sin(phi)=3/5, psi=ln(2)-(4/5)*atanh(12/25).
let ratio = (Real::from(3) / Real::from(5))?;
let psi = Real::from(2).ln()? - (Real::from(4) / Real::from(5))?
    * (Real::from(12) / Real::from(25))?.atanh()?;
let phi = psi.inverse_isometric_latitude(&ratio)?;
let latitude = Latitude::new(Angle::radians(phi.clone()), &mut context)?;
let tolerance: Real = "1e-30".parse()?;
let tolerance = tolerance.exact_rational().unwrap();
let [lo, hi] = (phi.sin() - ratio).certified_rational_interval(-128).unwrap();
assert!(lo >= -tolerance.clone() && hi <= tolerance);
let oblate = Arc::new(Ellipsoid::from_axes(
    Length::metres(5.into()), Length::metres(3.into()), &mut context,
)?);
let recovered = GeodeticPosition::new(
    latitude, origin.longitude().clone(), Length::metres(Real::zero()), oblate,
    origin.reference().clone(),
    InputProvenance::new("inverse equation example", InputAccuracy::ExactModel), &mut context,
)?;
let recovered_ecef = recovered.to_ecef(&mut context)?;
let recovered_frame = LocalFrame::new(&recovered, LocalAxes::Enu, &mut context)?;
assert_eq!(recovered_frame.to_local_position(&recovered_ecef, &mut context)?
    .receipt().certainty(), Certainty::Exact);
# Ok::<(), hyperspatial::Error>(())
```

Latitude validation has explicit policy and typed failure. Longitude wrapping
is opt-in; an unrolled pole longitude determines the local meridian. Local
coordinates belong to a particular prepared anchor; cloning a frame preserves
that identity. Moving to a different anchor requires conversion through ECEF.
Vector transforms rotate without the position origin translation.
The inverse selects the closest point on the ellipsoid, including ordinary
negative heights. It reports ambiguity at the centre, on the oblate equatorial
medial disk and on the prolate axial medial segment. Their unique cusp endpoints
remain supported. At a pole, `PoleMeridian::Use(longitude)` explicitly chooses
the meridian for subsequent local frames; `Reject` returns a typed ambiguity.
Forward and inverse batch APIs return complete output or an error.
`SphericalMercator` uses the source ellipsoid's equatorial radius and preserves
ellipsoidal height. Its parameters include scale, central meridian and false
origin. Longitude policy explicitly preserves turns, rejects outside the
principal strip, or wraps relative to the central meridian. Poles return a
typed singularity; Web tile clipping is not implicit. Prepared projections
retain their identity, model, source coordinates and certainty through inversion.
`Angle` and `Length` equality follows `Real` structural equality. Use native
certified scalar comparisons when mathematical equality of different expressions
is required.

`Limits` bounds work checkpoints, batch length and evidence depth. These are
not scalar heap-byte or wall-clock bounds. Cancellation is cooperative between
native calls and cannot change the meaning of an already returned coordinate.

Ellipsoidal Mercator, Transverse Mercator/UTM, geodesics, datum operations,
serialization, celestial/observer transformations
and ephemerides remain under development.
No physical accuracy is inferred from exact evaluation of a model. Raw Cartesian
ingress requires an explicit `InputProvenance`; missing uncertainty stays unknown.

- [Development plan](DEVELOPMENT.md): scope, ownership, proposed API, stages,
  and acceptance criteria.
- [Rust ecosystem comparison](docs/ecosystem.md): available crates and their
  relationship to the existing workspace.
- [Earth and sky research](docs/astronomy.md): celestial frames, time scales,
  observers, ephemerides, and reference workflows.
- [Scaffold admission record](docs/admission/2026-10-18-scaffold.md): the initial
  change contract and executed validation.
- [Native E1 admission record](docs/admission/e1-native.md): current forward/local
  contracts, validation evidence and remaining work.
- [Testing integration](docs/admission/testing.md): compiler/target gates,
  production coverage, source-consumer and fuzz checks.
- [Closest-point inverse admission](docs/admission/e1-inverse.md): inverse
  selection, exact equations, independent fixtures and qualified oracle results.
- [Representation and directed-testing admission](docs/admission/e1-directed-testing.md):
  22 scalar recipes, 484 ordered pairs, cache configurations, dispatch traces
  and sanitized campaigns.
- [Spherical Mercator admission](docs/admission/e2-mercator.md): projection
  domains, source identities, independent fixtures and development evidence.
- [Inverse-isometric scalar admission](../hyperreal/docs/admission-inverse-isometric-latitude.md):
  native oblate/prolate inversion, independent MPFR checks and Earth/local composition.

From the repository root:

```sh
bash crates/hyperspatial/scripts/checks.sh
bash crates/hyperspatial/scripts/representation-checks.sh
python3 crates/hyperspatial/scripts/coverage.py
bash scripts/bench-reports.sh hyperspatial
```

CI runs the checks on stable Rust and the declared 1.95 MSRV, and compiles the
native library for `wasm32-unknown-unknown`. Coverage requires the rustup
`llvm-tools-preview` component and measures production lines/regions/functions
from library and integration tests. It enforces a 95% line floor; branch
coverage and the 100% target remain open. The WASM build does not establish a
JavaScript binding or runtime consumer. Fuzz instructions are in
[fuzz/README.md](fuzz/README.md).
The optional `dispatch-trace` feature enables native Hyper-stack diagnostics;
it introduces no alternate numerical implementation. JSON and representation
serialization support used by the development probes are dev dependencies.

All changes follow the workspace [properties](../../properties.md) and
[admission protocol](../../docs/admission.md). The package uses the repository's
MIT license; see [LICENSE](./LICENSE).
The inverse algebra adapts GeographicLib's MIT-licensed equations; its
[upstream notice](LICENSE-GEOGRAPHICLIB) is retained. `map_3d` is a pinned
development-only oracle; it is absent from the normal dependency graph.
