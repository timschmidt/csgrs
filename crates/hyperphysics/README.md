<h1>
  hyperphysics
  <img src="./doc/hyperphysics.png" alt="hyperphysics logo" width="144" align="right">
</h1>

`hyperphysics` owns exact-aware physical carriers for the Hyper ecosystem. It records
materials, property assertions, bodies, fixtures, shapes, mass properties, contact
reports, field/process handoffs, and residual replay surfaces over `hyperreal::Real`
and `hyperlattice::Vector3` values.

The crate is not a replacement runtime physics engine. It is an adapter and
certification layer where authored physical facts stay visible before approximate
simulation, field, or engine exports are trusted.

## Hyper Ecosystem

`hyperphysics` connects exact geometry facts to physical interpretation.

- [hyperreal](https://github.com/timschmidt/hyperreal) and
  [hyperlattice](https://github.com/timschmidt/hyperlattice): scalar and vector values
  for material, shape, mass, and field reports.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact contact, sidedness, and
  incidence predicate policy.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypertri](https://github.com/timschmidt/hypertri), and
  [hypermesh](https://github.com/timschmidt/hypermesh): geometry/topology owners for
  shapes and mesh facts.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual replay and future
  coupled solver certification.
- [hypercircuit](https://github.com/timschmidt/hypercircuit),
  [hyperpath](https://github.com/timschmidt/hyperpath), and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): circuit, routing, and
  manufacturing context for coupled physical fixtures.

## Typical Physics Problems

Physics engines optimize throughput and stability under finite time steps. Contact
manifolds, collision margins, mass properties, material data, and constraint solvers
often combine approximation policy with geometry cleanup. That is useful at runtime, but
it makes it hard to audit whether a simulation lost a constraint because of tolerance,
mesh repair, or a real modeling issue.

`hyperphysics` keeps authored physical facts separate from runtime proposals. Exact
material and geometry-derived reports are retained at setup time, lossy exports are
named, and coupled or simulated states should be accepted only after exact residual or
diagnostic replay where possible.

## Main Types

- `ExactMaterial`, `MaterialPropertyGraph`, `MaterialAssertion`, and
  `ResolvedPropertyReport` store source-attributed material facts.
- `ExactBody3`, `ExactFixture3`, `PhysicsShape3`, `ClosedTriangleMesh3`,
  `AxisAlignedBox3`, `Plane3`, `Ray3`, `Segment3`, and `Triangle3` describe physical
  shapes and fixtures.
- `MassPropertyReport3`, `SymmetricInertia3`, and `MassPropertyCertificate3` report
  exact uniform-density mass properties.
- `ContactMaterial`, `AabbContactReport3`, and contact classification types describe
  current contact evidence.
- `ForceAccumulator3`, `StepReplayReport3`, `SystemDiagnostics3`, and
  `HypersolveResidualReplayReport` record replay and diagnostics.
- Thermal, optical, electromagnetic, photochemical, reaction-diffusion, and fluid
  modules define exact-aware handoff/report carriers for future solvers.

## Precision Model

Physical setup data uses `Real` and exact vectors. Mesh mass properties are computed
from oriented triangle decompositions using exact arithmetic. Material reports preserve
exact values, exact intervals, explicit unknowns, conflicts, and external replacement
status. Contact and shape reports prefer exact classification or explicit uncertainty
over tolerance inflation.

Primitive floats belong at rendering, external simulation-engine, file IO, diagnostics,
or adapter boundaries. They are not physical truth inside the crate.

## Performance Model

`hyperphysics` preserves cheap object facts so future adapters can avoid unnecessary
exact or simulation work: body class, shape kind, AABB bounds, support maps,
mass/inertia structure, material category, and coupling policy. Exact setup reports are
small enough to replay in tests and CI, while runtime-heavy work such as contact
manifold generation, FEM/FVM/FDTD/SPH evolution, and engine bridges remains outside the
core carrier layer.

Performance improvements should come from prepared shape facts, broad-phase bounds,
specialized exact queries, and explicit lossy adapters rather than hidden primitive
predicates.

## Current Status

Implemented today:

- exact material IDs, density validation, property graphs, and elastic derivation;
- exact bodies, fixtures, closed meshes, AABBs, planes, rays, segments, support maps,
  and point/query reports;
- exact uniform-density mass properties for closed triangle meshes;
- contact material validation and AABB contact classification;
- force accumulation, explicit step replay, momentum, and kinetic-energy diagnostics;
- `hypersolve` residual replay rows for coupled candidates;
- thermal, optical, electromagnetic, photochemical, reaction-diffusion, and fluid
  handoff/report carriers.

Known limits: broad contact generation, impulse solving, continuous collision, richer
mesh validation, and full field/fluid/thermal evolution are future certified solver or
adapter work.

## Installation

```toml
[dependencies]
hyperphysics = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperphysics = { path = "../hyperphysics" }
```

## Usage

Create exact setup facts, then hand simulation or field work to explicit adapters:

```rust,ignore
use hyperlattice::Vector3;
use hyperphysics::{
    AxisAlignedBox3, BodyId, BodyKind, ExactBody3, ExactFixture3, ExactMaterial,
    MaterialId, PhysicsShape3,
};
use hyperreal::Real;

let material = ExactMaterial::new(
    MaterialId::new("aluminum")?,
    "aluminum",
    Real::from(2700),
)?;

let shape = PhysicsShape3::AxisAlignedBox(AxisAlignedBox3::new(
    Vector3::new([Real::from(0), Real::from(0), Real::from(0)]),
    Vector3::new([Real::from(1), Real::from(1), Real::from(1)]),
)?);

let fixture = ExactFixture3::new("fixture-0")?.with_shape(shape);
let body = ExactBody3::new(BodyId::new("body-0")?, BodyKind::Rigid, vec![fixture]);
```

Mass properties, contact reports, thermal/optical/electromagnetic/fluid carriers, force
accumulators, and `hypersolve` residual replay rows all keep authored physical facts
separate from runtime engine proposals.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench mass_properties
```
