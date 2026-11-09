# HyperBRep

HyperBRep is the Hyper workspace's exact boundary-representation kernel for CAD.
It retains spatial geometry, parameter charts and topology in an immutable
`Model`. `hyperreal::Real` is the authoritative scalar; display meshes and STEP
encodings are derived consumers with their own contracts.

## Architecture

```text
Real and selected parameters
          │
Curve3 / Pcurve / Surface
          │
     ModelBuilder ── validation ──► immutable Model
                                      │
                              BodySet / SheetSet
                                      │
                         supported modeling operations
```

Hyperreal owns scalars, Hyperlimit owns certified predicates, Hypersolve owns
algebraic solving, and Hypercurve owns planar geometry. HyperBRep owns spatial
carriers, chart correspondence and validated topology. HyperBRep IO owns
[STEP syntax and mapping](../hyperbrep-io/README.md).

Vertices, edges, edge uses, wires, faces, shells and solids are borrowed through
model-bound references. Builders reject foreign keys and invalid incidence.
Whole edge/pcurve/surface image checks precede model publication; matching
endpoints or display samples are insufficient.

## Build and use

From the [CSGRS source checkout](../../readme.md), run:

```sh
cargo run -p hyperbrep --example basic
```

The runnable [basic example](examples/basic.rs) constructs and measures a cuboid.
Its essential construction flow is:

```rust
use hyperbrep::{Point3, Real, builder};

let bodies = builder::cuboid(
    Point3::origin(),
    Point3::new(Real::from(2), Real::from(3), Real::from(5)),
)?;
let exact_volume = bodies.bodies().next().expect("one cuboid").volume()?;
# let _ = exact_volume;
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Current boundary

The [support matrix](docs/SUPPORT_MATRIX.md) describes completed native workflows
and explicit blockers. The [implementation ledger](docs/IMPLEMENTATION_STATUS.json)
records the same work; constructor or carrier inventories do not prove closure.
The [architecture](docs/ARCHITECTURE.md) explains ownership and proof boundaries.

`BodySet` Boolean operations consume and return body selections, including
canonical empty and multiple-body results. Selected planar, analytic, spline
and retained-source workflows support the operations declared in that matrix.
A selected point or parameter need not have independent scalar coordinates.
Checked native replay reconstructs the required geometry and topology evidence;
see the [archive contract](docs/SELECTED_ARCHIVE.md).

General algebraic surface arrangements, unrestricted generated-geometry closure,
full work/heap/cancellation bounds and released-package qualification remain
open. Refinement/promotion budgets do not bound all computation or memory.
Numerical certainty, topological validity and source fidelity remain distinct.
The [development plan](docs/DEVELOPMENT_PLAN.md) and
[curve/surface closure plan](docs/CURVE_SURFACE_CLOSURE_PLAN.md) define the targets.

## Development checks

Run these commands from `crates/hyperbrep/`:

```sh
cargo fmt --all -- --check
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
```

Extensions follow the repository [admission protocol](../../docs/admission.md).
Earlier certification and migration narratives are preserved in the
[historical README snapshot](docs/history/README-2026-11-09.md), rather than used
as a second current capability ledger.
