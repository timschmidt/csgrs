# SimCore native-domain parity

This matrix tracks the parts of `simcore_language_spec.pdf` that map directly
to HyperCircuit's retained circuit, hierarchy, schematic, package, simulation,
and interchange domains. “Parity” here means equivalent semantic capability;
it does not mean accepting SimCore source syntax.

## Included requirements

| ID | SimCore requirement | HyperCircuit semantic mapping | Implementation evidence | Conformance evidence | Status |
|---|---|---|---|---|---|
| HC-SC-01 | Hierarchical module composition and one designated top level (§§4.2, 8, 9) | `Circuit`, `SubcircuitInstance`, and `CircuitLibrary::root`; deterministic validation and flattening | `src/model.rs`, `src/hierarchy.rs` | `tests/hierarchy.rs` | Verified |
| HC-SC-02 | Named input/output boundaries with retained direction (§§4.1–4.3, 8) | `CircuitPort`, `PortDirection`, optional-port validation, and explicit parent bindings | `src/model.rs`, `src/hierarchy.rs`, `src/erc.rs` | `tests/hierarchy.rs`, `tests/erc.rs` | Verified |
| HC-SC-03 | Scalar real, logic, and bus port types (§§4.1, 4.4.9, 6) | `PortSignalType::{Real, Logic, Bus}`, `CircuitPortType`, compatibility-default real ports, and cross-hierarchy type checking | `src/model.rs`, `src/hierarchy.rs` | `typed_hierarchy_rejects_incompatible_signal_shapes_on_one_parent_net` | Verified |
| HC-SC-04 | Ordered signal bundles and directional modport views (§§4.2, 6) | Typed `SignalBundleMember`, nominal `SignalBundle`, complete `Modport` views, and dual direction semantics | `src/interface.rs` | `tests/interface.rs` | Verified |
| HC-SC-05 | Interface/modport endpoints bind bundle members to module ports (§§6, 6.1) | `SignalBundleEndpoint`, `BundlePortBinding`, complete-member validation, port direction/type checking, and checked lowering to `SubcircuitPortBinding` | `src/interface.rs` | `dual_bundle_endpoints_lower_to_existing_hierarchy_port_bindings`, `typed_bundle_members_must_match_their_bound_circuit_ports` | Verified |
| HC-SC-06 | Explicit, identifiable connectivity rather than duplicated embedded connections (§4.4.7) | Authoritative `Net` topology, explicit `PinBinding`/`SubcircuitPortBinding`, stable `SchematicWireId`, and typed schematic endpoints | `src/model.rs`, `src/hierarchy.rs`, `src/schematic.rs` | `tests/declarative.rs`, `tests/hierarchy.rs`, `tests/schematic.rs` | Verified |
| HC-SC-07 | Ordered buses and retained subsets (§§4.4.9, 6) | `Bus` and `BusSlice` retain ordered net membership, slice direction, bounds, and validation | `src/model.rs` | `tests/hierarchy.rs`, `tests/edit.rs` | Verified |
| HC-SC-08 | Named parameters, exact values, units, defaults, and instance overrides (§§4.1, 4.4.12, 8) | `CircuitParameter`, `CircuitModuleParameter`, typed targets, exact `Real` defaults, units, provenance, and nested overrides | `src/model.rs`, `src/hierarchy.rs` | parameter tests in `tests/hierarchy.rs` | Verified |
| HC-SC-09 | Reusable packages/libraries with dependency and release identity (§7) | Semver catalog, source/digest provenance, deterministic dependency resolution, locks, portable part artifacts, and portable hierarchical circuit artifacts | `src/package.rs` | `tests/package.rs` | Verified |
| HC-SC-10 | Structural/electrical validation before execution (§§6–9) | Local circuit validation, hierarchy validation, bundle validation, schematic validation, PCB validation, and ERC | `src/model.rs`, `src/hierarchy.rs`, `src/interface.rs`, `src/schematic.rs`, `src/erc.rs` | corresponding test modules | Verified |
| HC-SC-11 | Structure is independent from selected behavior (§2, §10.2) | Retained topology is independent of `BehaviorRuntime`, event handlers, numerical adapters, and exact event agenda | `src/behavior.rs`, `src/event_simulation.rs`, `src/adapter.rs` | `tests/behavior.rs`, `tests/event_simulation.rs` | Verified |
| HC-SC-12 | Design-level simulation solver/integration intent, timestep, and stop time (§§4.4.6, 9, 13.4) | Circuit `AdapterKind`/`TransientPolicy` plus serializable, validated `TransientRunPolicy` with exact start/stop/step bounds and fixed/adaptive control | `src/model.rs`, `src/simulation.rs`, `src/interchange.rs` | `tests/simulation.rs`, native parity round-trip in `tests/interchange.rs` | Verified |
| HC-SC-13 | Exact block/wire geometry and canvas routing settings (§§4.4.4–4.4.5, 4.4.7) | Exact hierarchical block/symbol/port positions and sizes, wire endpoints and waypoints, connection style, grid/snap/display intent, paper intent, and validation | `src/schematic.rs` | `tests/schematic.rs`, native parity round-trip in `tests/interchange.rs` | Verified |
| HC-SC-14 | Wire name, color, width, style, logging, name visibility, and manual-route intent (§4.4.7.3, §13.5) | Sparse `SchematicWireMetadata` keyed by stable wire ID, separate from authoritative connectivity | `src/schematic.rs`, `src/interchange.rs` | native parity round-trip in `tests/interchange.rs` | Verified |
| HC-SC-15 | Machine-readable semantic interchange for the complete design | Schema v28 retains the root and child definitions, typed ports/bundles, event traces, transient run policy, schematic presentation, schematic, and PCB with migration and validation | `src/interchange.rs` | `tests/interchange.rs` | Verified |

## Deliberately excluded adapter concerns

The following are not native HyperCircuit parity requirements: `.sc`/`.scpkg`
lexing, RPV grammar, decorators as syntax, source spans/comments and
round-trip source serialization, generic structs/enums/classes/functions, DPI
file resolution, C/Python/Rust autocode, GUI window/zoom/pan state, TCP, LSP,
AI features, and unresolved future language proposals. Forward Euler/RK
solver names are likewise not copied into the circuit IR: HyperCircuit retains
its electrically meaningful adapter and integration policies while providing
the equivalent design-level run bounds and timestep configuration.

## Verification gate

Parity is complete only after:

1. every included row has direct implementation and conformance-test evidence;
2. default-feature tests pass;
3. `interchange` and all applicable feature combinations compile and test;
4. formatting and Clippy checks pass without warnings; and
5. the resulting commit is pushed.

## Verification record

Verified on 2026-07-25:

- `cargo test` passed.
- Focused `interchange`, hierarchy, interface, package, authoring, editor, and
  release-workflow conformance tests passed.
- `cargo clippy --all-targets --all-features -- -D warnings` passed.
- The complete all-feature suite passed with only
  `declarative_board_materializes_source_addressable_copper_and_drills`
  skipped. That pre-existing materialization assertion independently fails
  against the dirty sibling `hypermesh` worktree (`main` ahead by one commit
  with local changes) and is unrelated to these HyperCircuit semantics.
