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
- `SourceSpec`, `PropertyValue`, `ExternalReplacementStatus`, and certification reports
  make material-property provenance, conflicts, unknowns, and external proposals visible.

## Precision Model

Physical setup data uses `Real` and exact vectors. Mesh mass properties are computed
from oriented triangle decompositions using exact arithmetic. Material reports preserve
exact values, exact intervals, explicit unknowns, conflicts, and external replacement
status. Contact and shape reports prefer exact classification or explicit uncertainty
over tolerance inflation.

Primitive floats belong at rendering, external simulation-engine, file IO, diagnostics,
or adapter boundaries. They are not physical truth inside the crate.

Numerical explosion is controlled by retaining physical facts as compact source objects:
materials, fixtures, support maps, AABBs, oriented mesh integrals, property graphs, and
residual rows. The crate does not turn every physical carrier into a dense sampled field;
field, fluid, thermal, EM, and process adapters must report what they approximate.

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

The measured reference audit, retained/rejected experiments, and exact dispatch-trace
protocol are recorded in [PERFORMANCE.md](PERFORMANCE.md).

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
hyperphysics = "0.3.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperphysics = { path = "../hyperphysics" }
```

## Usage

Create exact setup facts, then hand simulation or field work to explicit adapters:

```rust,no_run
use hyperlattice::Vector3;
use hyperphysics::{
    AxisAlignedBox3, BodyId, BodyKind, ExactBody3, ExactFixture3, ExactMaterial,
    FixtureId, MaterialId, PhysicsShape3,
};
use hyperreal::Real;

fn main() -> hyperphysics::PhysicsResult<()> {
    let material = ExactMaterial::new(
        MaterialId::new("aluminum")?,
        "aluminum",
        Real::from(2700),
    )?;

    let shape = PhysicsShape3::AxisAlignedBox(Box::new(AxisAlignedBox3::new(
        Vector3::new([Real::from(0), Real::from(0), Real::from(0)]),
        Vector3::new([Real::from(1), Real::from(1), Real::from(1)]),
    )?));

    let fixture = ExactFixture3::new(FixtureId::new("fixture-0")?, shape, material);
    let body = ExactBody3::new(BodyId::new("body-0")?, BodyKind::Dynamic, vec![fixture]);
    assert_eq!(body.fixtures().len(), 1);
    Ok(())
}
```

Exact contact and replay reports keep runtime proposals auditable:

```rust,no_run
use hyperlattice::Vector3;
use hyperphysics::{
    AabbContactReport3, AxisAlignedBox3, ContactClassification, ForceAccumulator3,
    ForceContribution3, StepReplayReport3,
};
use hyperreal::Real;

fn main() -> hyperphysics::PhysicsResult<()> {
    let left = AxisAlignedBox3::new(
        Vector3::new([Real::from(0), Real::from(0), Real::from(0)]),
        Vector3::new([Real::from(1), Real::from(1), Real::from(1)]),
    )?;
    let right = AxisAlignedBox3::new(
        Vector3::new([Real::from(1), Real::from(0), Real::from(0)]),
        Vector3::new([Real::from(2), Real::from(1), Real::from(1)]),
    )?;
    let contact = AabbContactReport3::classify(&left, &right)?;
    assert_eq!(contact.classification, ContactClassification::Touching);

    let mut forces = ForceAccumulator3::default();
    forces.push(ForceContribution3 {
        source: "test force".into(),
        force: Vector3::new([Real::from(1), Real::from(0), Real::from(0)]),
    });
    let step = StepReplayReport3::symplectic_euler_replay(
        Real::from(2),
        Real::from(1),
        Vector3::zero(),
        Vector3::zero(),
        &forces,
    )?;
    assert!(step.exact_replay);
    Ok(())
}
```

Mass properties, contact reports, thermal/optical/electromagnetic/photochemical/fluid
carriers, force accumulators, and `hypersolve` residual replay rows all keep authored
physical facts separate from runtime engine proposals.

## References

The implementation comments describe local invariants; the scientific and numerical
background is consolidated here.

- Bender, Jan, and Dan Koschier. "Divergence-Free Smoothed Particle Hydrodynamics."
  *Proceedings of SCA*, 2015, https://doi.org/10.1145/2786784.2786796.
- Beer, August. "Bestimmung der Absorption des rothen Lichts in farbigen
  Flüssigkeiten." *Annalen der Physik und Chemie*, 1852,
  https://doi.org/10.1002/andp.18521620505.
- Carslaw, H. S., and J. C. Jaeger. *Conduction of Heat in Solids*. 2nd ed., Oxford
  University Press, 1959.
- Fick, Adolf. "Ueber Diffusion." *Annalen der Physik*, vol. 170, no. 1, 1855,
  https://doi.org/10.1002/andp.18551700105.
- Fourier, Joseph. *The Analytical Theory of Heat*. Cambridge University Press, 1878.
- Gilbert, Elmer G., Daniel W. Johnson, and S. Sathiya Keerthi. "A Fast Procedure for
  Computing the Distance Between Complex Objects in Three-Dimensional Space."
  *IEEE Journal on Robotics and Automation*, vol. 4, no. 2, 1988,
  https://doi.org/10.1109/56.2083.
- Ihmsen, Markus, et al. "Implicit Incompressible SPH." *IEEE Transactions on
  Visualization and Computer Graphics*, vol. 20, no. 3, 2014,
  https://doi.org/10.1109/TVCG.2013.105.
- Jacobs, Paul F. *Rapid Prototyping & Manufacturing: Fundamentals of
  Stereolithography*. Society of Manufacturing Engineers, 1992.
- Lambert, Johann Heinrich. *Photometria*. 1760.
- Landau, L. D., and E. M. Lifshitz. *Theory of Elasticity*. 3rd ed., Butterworth-
  Heinemann, 1986.
- Marsden, Jerrold E., and Matthew West. "Discrete Mechanics and Variational
  Integrators." *Acta Numerica*, vol. 10, 2001,
  https://doi.org/10.1017/S096249290100006X.
- Maxwell, James Clerk. "A Dynamical Theory of the Electromagnetic Field."
  *Philosophical Transactions of the Royal Society of London*, 1865,
  https://doi.org/10.1098/rstl.1865.0008.
- Mirtich, Brian. "Fast and Accurate Computation of Polyhedral Mass Properties."
  *Journal of Graphics Tools*, vol. 1, no. 2, 1996,
  https://doi.org/10.1080/10867651.1996.10487458.
- Monaghan, J. J. "Smoothed Particle Hydrodynamics." *Annual Review of Astronomy and
  Astrophysics*, vol. 30, 1992, https://doi.org/10.1146/annurev.aa.30.090192.002551.
- Stewart, David E., and Jeffrey C. Trinkle. "An Implicit Time-Stepping Scheme for
  Rigid Body Dynamics with Inelastic Collisions and Coulomb Friction."
  *International Journal for Numerical Methods in Engineering*, vol. 39, no. 15,
  1996, https://doi.org/10.1002/(SICI)1097-0207(19960815)39:15%3C2673::AID-NME972%3E3.0.CO;2-I.
- Stratton, Julius Adams. *Electromagnetic Theory*. McGraw-Hill, 1941.
- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
  vol. 7, nos. 1-2, 1997, https://doi.org/10.1016/0925-7721(95)00040-2.

## Development

```sh
cargo fmt --all -- --check
cargo test --locked
cargo check --benches --locked
cargo clippy --all-targets --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo bench --bench mass_properties
```

## Hyper Ecosystem

`hyperphysics` builds on [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlattice](https://github.com/timschmidt/hyperlattice), and
[hyperlimit](https://github.com/timschmidt/hyperlimit), consumes shape facts from
[hypermesh](https://github.com/timschmidt/hypermesh), and replays coupled residuals
through [hypersolve](https://github.com/timschmidt/hypersolve). Related physical
contexts live in [hypercircuit](https://github.com/timschmidt/hypercircuit),
[hyperpath](https://github.com/timschmidt/hyperpath), and
[hyperparts](https://github.com/timschmidt/hyperparts).
