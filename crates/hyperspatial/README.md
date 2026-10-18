# hyperspatial

`hyperspatial` is the planned geospatial and astronomical coordinate
transformation layer of the Hyper geometry workspace. Its first development
target is Earth geodesy, map projections, and local reference frames. Later
work connects terrestrial positions to celestial frames, observers, and
ephemerides.

The package currently contains an empty library target and a development plan.
It exposes no transformation APIs, has no dependencies, and is not publishable.
The version `0.1.0` identifies this local scaffold, not a released capability.

Native implementations will reuse `hyperreal`, `hyperlattice`, and `hyperlimit`.
All production numerical quantities, including epochs and durations, will use
`hyperreal::Real` or types backed by it. External crates are development-only
test/benchmark oracles and algorithmic inspiration.
Results must preserve exact input meaning and distinguish numerical certainty
from model approximation, measurement uncertainty, and dataset provenance.

- [Development plan](DEVELOPMENT.md): scope, ownership, proposed API, stages,
  and acceptance criteria.
- [Rust ecosystem comparison](docs/ecosystem.md): available crates and their
  relationship to the existing workspace.
- [Earth and sky research](docs/astronomy.md): celestial frames, time scales,
  observers, ephemerides, and reference workflows.
- [Scaffold admission record](docs/admission/2026-10-18-scaffold.md): the initial
  change contract and executed validation.

From the repository root:

```sh
cargo check --locked -p hyperspatial --all-targets --all-features
cargo test --locked -p hyperspatial --all-features
cargo clippy --locked -p hyperspatial --all-targets --all-features -- -D warnings
cargo doc --locked -p hyperspatial --no-deps --all-features
```

All changes follow the workspace [properties](../../properties.md) and
[admission protocol](../../docs/admission.md). The package uses the repository's
MIT license; see [LICENSE](LICENSE).
