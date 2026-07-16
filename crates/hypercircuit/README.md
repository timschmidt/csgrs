# hypercircuit

`hypercircuit` stores exact-aware circuit models, Modified Nodal Analysis (MNA)
stamps, residual replays, nonlinear-device policy, and multiphysics coupling
reports. Circuit truth uses `hyperreal::Real`; numerical solvers are proposal
engines whose candidates can be checked against the authored equations.

The crate is not a complete SPICE simulator. Its current executable path builds
small dense linear MNA systems and certifies proposed solutions. Nonlinear,
transient, sparse, and coupled-field solvers remain explicit adapter boundaries.

## Installation

```toml
[dependencies]
hypercircuit = "0.3.0"
```

Use a sibling checkout during Hyper-stack development:

```toml
[dependencies]
hypercircuit = { path = "../hypercircuit" }
```

## Quick start

Build a one-node conductance problem and replay an exact candidate:

```rust
use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitResult, ComponentId, LinearStamp,
    Net, NetId, Real, TransientPolicy,
};

fn main() -> CircuitResult<()> {
    let out = NetId::new("out")?;
    let circuit = Circuit::new(
        CircuitId::new("conductance")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: out.clone(),
        is_ground: false,
    })
    .with_stamp(LinearStamp::Conductance {
        component: ComponentId::new("g1")?,
        part: None,
        pos: Some(out),
        neg: None,
        conductance: Real::from(2),
    });

    let system = circuit.linear_mna_system()?;
    let replay = system.replay_candidate(&[Real::zero()])?;
    assert!(replay.accepted);
    Ok(())
}
```

## Core API

- `Circuit`, `Net`, `CircuitInstance`, `DeviceModel`, and the ID types preserve
  topology, model parameters, and stable identity before solver lowering.
- `LinearStamp`, `MnaProblem`, `MnaUnknown`, and `LinearMnaSystem` build exact
  dense MNA equations. `LinearMnaSystem::replay_candidate` computes `A*x - b`
  with exact values and returns a `ResidualReplayReport`.
- `NonlinearDeviceReport`, `PiecewiseLinearSegment`, `EventPolicy`, and
  `SwitchState` record device domains and event decisions without hiding them in
  a numerical tolerance.
- `PhysicalElectricalPort`, `ThermalPort`, `CoupledResidualBlock`, and
  `ElectrothermalRcReport` describe circuit/physics boundaries. Call
  `CoupledResidualBlock::to_hypersolve_problem` to hand residual rows to
  `hypersolve`.
- `CircuitAdapterReport` records the solver, tolerance policy, and exact-replay
  result of an external numerical adapter.

## Precision and performance

Primitive floats belong only in named import, diagnostic, or solver-adapter
boundaries. Exact MNA rows, unknown ordering, model parameters, domains, and
coupling residuals remain structured so they can be replayed without treating a
solver tolerance as proof. An unknown replay is reported as uncertainty rather
than silently accepted.

The dense implementation is intended for small fixtures and certification
paths. Stable IDs and the separation between stamp construction and replay let
future sparse, transient, and DAE adapters reuse semantic context without
rebuilding it. The crate does not eagerly expand nonlinear devices or transient
policies into a global expression tree.

Currently implemented: linear conductance, current-source, voltage-source,
VCCS, and transient-companion stamps; exact dense residual replay; nonlinear
and switch report carriers; electrothermal RC reports; and `hypersolve`
coupling handoff. Full diode/MOSFET laws, Newton iteration, sparse matrices,
time integration, and field solvers are not yet implemented here.

Duplicate net-voltage and branch-current unknowns are rejected before assembly.
See [the reference and performance audit](PERFORMANCE.md) for the source-by-source
mapping, retained benchmark results, rejected trials, and validation evidence.

## References

- Ho, Ruehli, and Brennan, [*The Modified Nodal Approach to Network
  Analysis*](https://doi.org/10.1109/TCS.1975.1084079), 1975.
- Nagel, [*SPICE2: A Computer Program to Simulate Semiconductor
  Circuits*](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1975/9602.html),
  UCB/ERL M520, 1975.
- Yap, [*Towards Exact Geometric
  Computation*](https://doi.org/10.1016/0925-7721(95)00040-2), 1997.
- Cortes Garcia, De Gersem, and Schoeps, [*A Structural Analysis of
  Field/Circuit Coupled Problems Based on a Generalised Circuit
  Element*](https://doi.org/10.1007/s11075-019-00686-x), 2020.
- LLNL, [SUNDIALS IDA](https://computing.llnl.gov/projects/sundials/ida), the
  DAE/BDF adapter model referenced by `TransientPolicy::IdaDaeAdapter`.

Direct dependencies: [hyperreal](https://github.com/timschmidt/hyperreal) ·
[hypersolve](https://github.com/timschmidt/hypersolve). Related Hyper crates:
[hyperparts](https://github.com/timschmidt/hyperparts) ·
[hyperpath](https://github.com/timschmidt/hyperpath) ·
[hyperphysics](https://github.com/timschmidt/hyperphysics) ·
[hyperevolution](https://github.com/timschmidt/hyperevolution)

## Development

```sh
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cargo bench --bench mna
```
