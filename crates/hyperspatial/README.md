# hyperspatial

`hyperspatial` is the native geospatial and astronomical coordinate
transformation layer of the Hyper geometry workspace. Its first development
target is Earth geodesy, map projections, and local reference frames. Later
work connects terrestrial positions to celestial frames, observers, and
ephemerides.

The first implementation provides exact angular/length units, validated
spherical/oblate/prolate ellipsoids, geodetic-to-ECEF conversion, and anchored
ENU/NED position/vector transforms. Reference identity, coordinate epoch,
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
# Ok::<(), hyperspatial::Error>(())
```

Latitude validation has explicit policy and typed failure. Longitude wrapping
is opt-in; an unrolled pole longitude determines the local meridian. Local
coordinates belong to a particular prepared anchor; cloning a frame preserves
that identity. Moving to a different anchor requires conversion through ECEF.
Vector transforms rotate without the position origin translation.
`Angle` and `Length` equality follows `Real` structural equality. Use native
certified scalar comparisons when mathematical equality of different expressions
is required.

`Limits` bounds work checkpoints, batch length and evidence depth. These are
not scalar heap-byte or wall-clock bounds. Cancellation is cooperative between
native calls and cannot change the meaning of an already returned coordinate.

ECEF-to-geodetic inversion, projections, datum operations, serialization,
celestial/observer transformations and ephemerides remain under development.
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

From the repository root:

```sh
cargo check --locked -p hyperspatial --all-targets --all-features
cargo test --locked -p hyperspatial --all-features
cargo clippy --locked -p hyperspatial --all-targets --all-features -- -D warnings
cargo doc --locked -p hyperspatial --no-deps --all-features
```

All changes follow the workspace [properties](../../properties.md) and
[admission protocol](../../docs/admission.md). The package uses the repository's
MIT license; see [LICENSE](./LICENSE).
