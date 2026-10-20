# Changelog

## [0.3.0] - Unreleased

### Breaking

- `NonlinearDeviceKind::MosfetPlaceholder` and
  `NonlinearDeviceReport::mosfet_placeholder` are replaced by
  `NonlinearDeviceKind::Mosfet` and the level-one square-law model
  (`SquareLawMosfet`, `solve_square_law_mosfet_newton`).
- `Circuit` gains the public fields `buses`, `bus_slices`, `rails`, `ports`,
  `port_types`, `module_parameters`, `source_stimuli`, and `subcircuits`, and
  `DeviceModel` gains `pins`. Struct literals must set them; `Circuit::new` and
  the builder methods are unaffected.
- `LinearMnaSystem::from_stamps` returns `CircuitError::DuplicateUnknown` when
  an unknown is declared twice. It previously kept the later slot.
- `CircuitError` gains `DuplicateUnknown`, `MissingInstance`,
  `MissingConnectionNet`, `DuplicatePinBinding`, `SingularLinearSystem`,
  `IndeterminateLinearPivot`, and `LinearSolveArithmetic`, and
  `DeviceModelKind` gains `Diode` and `Mosfet`. Both enums are exhaustive, so
  `match` expressions over them must handle the new variants.

### Added

- Exact dense MNA solving: `LinearMnaSystem::solve_exact` eliminates with
  certified pivots and replays its own candidate, and
  `Circuit::linear_mna_from_devices` lowers device instances. `CircuitError`
  implements `Display` and `std::error::Error`.
- Transient simulation with exact source waveforms and breakpoints
  (`transient_step`, `transient_run`, `TransientSession`), a deterministic
  event agenda, and a behavior runtime. The `behavior-async` feature adds
  cooperative asynchronous behaviors.
- Nonlinear DC and AC analysis: piecewise-linear devices, Shockley diode and
  square-law MOSFET Newton solves whose results are replayed against the device
  law, and AC MNA systems and sweeps (`AcMnaSystem`).
- Hierarchy and reuse: `CircuitLibrary`, subcircuit instances and flattening,
  module parameters, typed signal bundles, and package catalogs and locks.
- Electrical rule checking (`Circuit::electrical_rule_check`), schematic
  layouts with automatic placement (`Circuit::auto_schematic`), SVG rendering,
  and KiCad schematic import and export.
- `layout` feature: PCB authoring (`PcbLayout`, stackups, land patterns,
  placement, routes, vias, zones, length and phase tuning, panels, and
  design-for-test intent), Hyperpath routing handoff, bounded negotiated
  autorouting, BOM and pick-and-place outputs, KiCad PCB import and export,
  and a tsCircuit `SimpleRouteJson` adapter.
- `geometry` and `drc` features: CSGRS materialization, fabrication outputs
  (Gerber X3, Gerber Job, Excellon, IPC-D-356), and HyperDRC readiness
  reports. Enabling both `drc` and `interchange` adds deterministic
  manufacturing release bundles. `LegacyCsgrsElectronicsImport` is
  migration-only and scheduled for removal in 0.4.0.
- `interchange` and `lceda` features: versioned `SemanticDocument` JSON,
  project manifests, edit history with undo and redo, KiCad library import,
  IPC-2581 inspection, and an EasyEDA/LCEDA Pro adapter.
- `hypercircuit` command-line tool (requires `drc` and `interchange`) with
  `check`, `ir`, `snapshot`, `bom`, `export-kicad`, `export-svg`, and
  `release build`/`release verify` commands.
- `dispatch-trace` feature, which forwards `hyperreal`'s exact-dispatch
  instrumentation.

### Changed

- XML parsing under `interchange` uses quick-xml 0.42, and the `drc` feature's
  HyperDRC dependency uses quick-xml 0.42 and calamine 0.37. These versions
  address RUSTSEC-2026-0194 and RUSTSEC-2026-0195 (quadratic duplicate-attribute
  checks and unbounded namespace allocation). Invalid UTF-8 in an XML name or
  attribute value is still reported as an XML error.
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.2.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names (the previous release named none).

For 0.2.0 and earlier, see [crates.io](https://crates.io/crates/hypercircuit/versions).
