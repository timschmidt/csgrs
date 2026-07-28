<h1>
  Hyperphysics
  <img src="./doc/hyperphysics.png" alt="Hyperphysics logo" width="144" align="right">
</h1>

Exact-aware physical setup, mass-property, collision-query, material, and
multiphysics handoff carriers for the Hyper ecosystem.

Hyperphysics keeps authored physical facts visible before approximate
simulation or field engines consume them. It owns material provenance, bodies,
fixtures, physical shape interpretation, exact mass properties, contact and
support-map reports, force/step replay, and report-bearing thermal, optical,
electromagnetic, photochemical, diffusion, and fluid interfaces.

It is not a real-time rigid-body engine or a full PDE solver. Runtime proposals
remain separate from certified setup and replay evidence.

This README describes crate version `0.3.0`.

## Primary types

| Type | Role |
| --- | --- |
| `ExactMaterial`, `MaterialId` | Material identity, density, and validation |
| `MaterialPropertyGraph`, `MaterialAssertion`, `SourceSpec` | Source-attributed physical properties |
| `ExactBody3`, `ExactFixture3`, `PhysicsShape3` | Body/fixture hierarchy and physical geometry |
| `ClosedTriangleMesh3`, `AxisAlignedBox3`, `Plane3`, `Ray3`, `Segment3`, `Triangle3` | Supported exact shape carriers |
| `MassPropertyReport3`, `SymmetricInertia3` | Exact uniform-density mass and inertia |
| `GjkQueryReport3`, `AabbContactReport3` | Convex and box contact/separation evidence |
| `ForceAccumulator3`, `StepReplayReport3`, `SystemDiagnostics3` | Exact force and integration replay |
| `HypersolveResidualReplayReport` | Exact replay of coupled residual rows |

## Install

```toml
[dependencies]
hyperphysics = "0.3.0"
```

There are no default features. `dispatch-trace` forwards exact-dispatch
instrumentation through the geometry and scalar stack.

## Quick start

This checked example creates an exact material, fixture, and dynamic body.

<!-- quickstart:start -->
```rust
use hyperlattice::Vector3;
use hyperphysics::{
    AxisAlignedBox3, BodyId, BodyKind, ExactBody3, ExactFixture3, ExactMaterial, FixtureId,
    MaterialId, PhysicsShape3,
};
use hyperreal::Real;

fn main() -> hyperphysics::PhysicsResult<()> {
    let material = ExactMaterial::new(MaterialId::new("aluminum")?, "aluminum", Real::from(2700))?;
    let bounds = AxisAlignedBox3::new(
        Vector3::new([Real::from(0), Real::from(0), Real::from(0)]),
        Vector3::new([Real::from(1), Real::from(1), Real::from(1)]),
    )?;
    let fixture = ExactFixture3::new(
        FixtureId::new("fixture-0")?,
        PhysicsShape3::axis_aligned_box(bounds),
        material,
    );
    let body = ExactBody3::new(BodyId::new("body-0")?, BodyKind::Dynamic, vec![fixture]);

    assert_eq!(body.fixtures().len(), 1);
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

## Ownership and evidence

```text
geometry carrier + ExactMaterial
              │
         ExactFixture3
              │
          ExactBody3
              │
      setup/query/replay reports
              │
 approximate runtime or field adapter
```

Geometry crates own geometric truth. Hyperphysics assigns physical meaning and
validates the evidence required for mass, collision, material, and coupling
operations. Primitive floats belong at engine, rendering, diagnostic, or file
boundaries.

## API guide

### Materials and properties

- `MaterialId::new` and `ExactMaterial::new` create validated material identity,
  name, and exact density.
- `SourceSpec::new` identifies an authority and locator.
- `PropertyValue::{exact_scalar, interval, external_proposal}` distinguishes
  exact facts, exact intervals, and externally proposed replacements.
- `MaterialPropertyGraph::{push, assertions, resolve}` retains competing
  `MaterialAssertion` values and reports agreement, conflict, unknowns, and
  external replacement status.
- `derive_isotropic_shear_modulus` derives supported elastic facts with source
  and equation evidence.
- `PhysicalPort` and `PhysicsCertificationReport` define explicit material and
  multiphysics handoff boundaries.

### Bodies, shapes, and mass

- `BodyId::new`, `FixtureId::new`, `ExactFixture3::new`, and
  `ExactBody3::new` build the body hierarchy.
- `Triangle3::new`, `ClosedTriangleMesh3::new`,
  `AxisAlignedBox3::new`, `Plane3::new`, `Ray3::new`, and `Segment3::new`
  construct supported exact shapes.
- `PhysicsShape3::{closed_triangle_mesh, axis_aligned_box,
  classification_report}` is the common physical shape carrier.
- `ClosedTriangleMesh3::to_hypermesh_exact` performs the explicit exact mesh
  conversion.
- `MassPropertyReport3::{zero, uniform_density_mass_properties,
  material_mass_properties}` computes exact oriented-triangle volume, center
  of mass, and symmetric inertia with a `MassPropertyCertificate3`.

### Classification and collision

- Shape `classify_point` methods report exact box, plane, or triangle point
  relations.
- `Plane3::{classify_ray, classify_segment}` reports supported ray/segment
  relations without tolerance inflation.
- `AxisAlignedBox3::{certified_disjoint, support_map}` exposes broad-phase and
  support evidence.
- `ExactSupportMap3` is the generic convex support interface.
  `gjk_query_3d` and `gjk_query_3d_with_config` return intersection,
  separation, closest simplex, Hypersolve projection evidence, iteration, and
  termination status.
- `AabbContactReport3::classify` distinguishes separated, touching, and
  overlapping boxes exactly.
- `ContactMaterial::new` validates restitution and friction properties.

### Forces, integration, and residual replay

- `ForceAccumulator3::{push, contributions, total_force}` retains individual
  authored force sources and their exact sum.
- `StepReplayReport3::{explicit_euler_replay, symplectic_euler_replay}`
  reproduces one exact integration step and records the selected policy.
- `SystemDiagnostics3::from_mass_velocity` reports exact momentum and kinetic
  energy where supported.
- `HypersolveResidualReplayReport::replay` evaluates coupled constraints and
  `all_residuals_zero` summarizes exact satisfaction.

These are replay and diagnostic APIs, not an open-ended time-stepping runtime.

### Thermal, optical, electromagnetic, process, and fluid carriers

- `ThermalMaterial::new`, `TemperatureField3::new`,
  `SteadySlabConductionReport::through_slab`,
  `HeatSource3::joule_heating`,
  `TransientThermalStepReport::energy_balance_step`, and
  `LumpedRcThermalStepReport::explicit_euler_step` provide exact-aware thermal
  setup and replay reports.
- `OpticalMedium`, `OpticalRay3`, and `OpticalInterface3` classify interfaces;
  `SnellNormalReport`, `FresnelNormalReport`, and
  `BeerLambertSlabReport::through_slab` expose supported optical calculations.
- `ElectromagneticMaterial::new`,
  `linear_isotropic_electric_response`, field regions, and boundary conditions
  retain electromagnetic setup and report status.
- `VatPhotopolymerWorkingCurve::replay`,
  `ReactionDiffusionTransport::diffusive_courant_report`, and related
  concentration/state carriers expose process-model decisions.
- `FluidMaterial`, `FluidParticle3`, `FluidBoundary3`, and
  `FluidFixture3::{with_particle, with_boundary, conservation_report}` retain
  fluid setup and conservation evidence.

These modules define validated inputs and small exact replay surfaces. They do
not claim full FEM, FVM, FDTD, SPH, reaction-diffusion, or optical-field
evolution.

## Guarantees and boundaries

- Physical setup values use `hyperreal::Real` and exact Hyperlattice vectors.
- Mesh mass properties use exact oriented triangle decompositions.
- Material reports preserve source, exact value or interval, conflicts, and
  replacement status.
- Contact reports prefer exact classification or explicit uncertainty over
  hidden margins.
- External engine state is a proposal until the required exact residual or
  diagnostic replay succeeds.
- Retained bodies, fixtures, AABBs, support maps, property graphs, and residual
  rows keep exact work compact instead of expanding every carrier into a
  sampled field.

Current collision support includes generic GJK over `ExactSupportMap3` and a
built-in AABB implementation. Penetration depth, contact manifolds, impulse
solving, continuous collision, and complete field evolution are outside the
current certified API.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Exact-dispatch instrumentation across scalar and geometry dependencies |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --all-features --locked
cargo clippy --all-targets --all-features --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features --locked
cargo check --benches --all-features
```

Measured benchmarks, dispatch-trace expectations, and the reference-guided
performance audit are recorded in [PERFORMANCE.md](PERFORMANCE.md). Fuzz
ownership and replay instructions live in [fuzz/README.md](fuzz/README.md).

## References

These sources cover the mechanics, collision, mass-property, and multiphysics
models represented by the crate:

- Gilbert, E. G., Johnson, D. W., and Keerthi, S. S. “A Fast Procedure for
  Computing the Distance Between Complex Objects in Three-Dimensional Space.”
  *IEEE Journal on Robotics and Automation* 4(2), 1988.
  [DOI: 10.1109/56.2083](https://doi.org/10.1109/56.2083).
- Mirtich, B. “Fast and Accurate Computation of Polyhedral Mass Properties.”
  *Journal of Graphics Tools* 1(2), 1996.
  [DOI: 10.1080/10867651.1996.10487458](https://doi.org/10.1080/10867651.1996.10487458).
- Marsden, J. E., and West, M. “Discrete Mechanics and Variational
  Integrators.” *Acta Numerica* 10, 2001.
  [DOI: 10.1017/S096249290100006X](https://doi.org/10.1017/S096249290100006X).
- Stewart, D. E., and Trinkle, J. C. “An Implicit Time-Stepping Scheme for
  Rigid Body Dynamics with Inelastic Collisions and Coulomb Friction.”
  *International Journal for Numerical Methods in Engineering* 39(15), 1996.
  [DOI](https://doi.org/10.1002/(SICI)1097-0207(19960815)39:15%3C2673::AID-NME972%3E3.0.CO;2-I).
- Landau, L. D., and Lifshitz, E. M. *Theory of Elasticity*, 3rd ed.
  Butterworth-Heinemann, 1986.
- Carslaw, H. S., and Jaeger, J. C. *Conduction of Heat in Solids*, 2nd ed.
  Oxford University Press, 1959.
- Maxwell, J. C. “A Dynamical Theory of the Electromagnetic Field.”
  *Philosophical Transactions of the Royal Society of London*, 1865.
  [DOI: 10.1098/rstl.1865.0008](https://doi.org/10.1098/rstl.1865.0008).
- Beer, A. “Bestimmung der Absorption des rothen Lichts in farbigen
  Flüssigkeiten.” *Annalen der Physik und Chemie*, 1852.
  [DOI: 10.1002/andp.18521620505](https://doi.org/10.1002/andp.18521620505).
- Fick, A. “Ueber Diffusion.” *Annalen der Physik* 170(1), 1855.
  [DOI: 10.1002/andp.18551700105](https://doi.org/10.1002/andp.18551700105).
- Monaghan, J. J. “Smoothed Particle Hydrodynamics.”
  *Annual Review of Astronomy and Astrophysics* 30, 1992.
  [DOI: 10.1146/annurev.aa.30.090192.002551](https://doi.org/10.1146/annurev.aa.30.090192.002551).
- Ihmsen, M., Cornelis, J., Solenthaler, B., Horvath, C., and Teschner, M.
  “Implicit Incompressible SPH.” *IEEE TVCG* 20(3), 2014.
  [DOI: 10.1109/TVCG.2013.105](https://doi.org/10.1109/TVCG.2013.105).
- Bender, J., and Koschier, D. “Divergence-Free Smoothed Particle
  Hydrodynamics.” *Proceedings of SCA*, 2015.
  [DOI: 10.1145/2786784.2786796](https://doi.org/10.1145/2786784.2786796).
- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

## Acknowledgements

Hyperphysics builds on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlattice](https://github.com/timschmidt/hyperlattice),
[Hyperlimit](https://github.com/timschmidt/hyperlimit),
[Hypermesh](https://github.com/timschmidt/hypermesh), and
[Hypersolve](https://github.com/timschmidt/hypersolve). The cited scientific
work informs the report models without implying source-code derivation.

## License and contributing

Licensed under the [Apache License 2.0](LICENSE).

Bug reports should include exact shape/material inputs, the operation, enabled
features, and the complete report. Before proposing a change, run formatting,
the focused regression, all-feature tests, and strict Clippy.
