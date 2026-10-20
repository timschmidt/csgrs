# Changelog

## [0.3.0] - Unreleased

Initial release.

Hyperparts records source-attributed part knowledge graphs and cross-domain
handoff records for the Hyper stack. It keeps part families and variants,
assertions, terminals, interfaces, compatibility, geometry and physics
handles, EDA intake, procurement facts, and manufacturing capabilities
without discarding provenance or uncertainty. It indexes facts; geometry,
circuit, physics, routing, DRC, and process-planning decisions belong to
their owning crates.

- `PartGraph`, `PartFamily`, and `PartVariant` form a queryable in-memory
  graph with validated identities (`PartId`, `VariantId`, `TerminalId`, and
  part-number types).
- `Assertion<T>` and `AssertionValue` keep facts attributed to a `SourceRef`
  and optional `SourceRevision`, with exact `hyperreal::Real` scalars,
  validated intervals, explicit unknowns, and conflicts.
- `Terminal`, `Interface`, and electronic records (`ElectronicPart`,
  `Pinout`, power domains, and supply rails) describe connection surfaces.
  `PartGraph::safe_connection` reports power-to-ground as unsafe, requires
  known power envelopes to overlap, and returns `ConnectionDecision::Unknown`
  when polarity or voltage facts are missing.
- `PartGraph::query_parts` and the domain-specific query types return ranked
  candidates with separate evidence, conflicts, and unknowns.
- `GeometryHandoffReport` and `PhysicsHandoffReport` reference geometry and
  physics artifacts by stable handles and report downstream readiness.
  Capability, tool, and manufacturing-route records support direct lookup
  without claiming toolpath or schedule validation.
- `import_eda_authoring_bundle` imports Circuit-JSON-like EDA records into a
  populated graph. `EdaExactField` preserves the original decimal spelling,
  and `EdaAuthoringImportResult::is_exact_ready` keeps missing, rejected,
  lossy, and review-only fields visible.
- There are no default features; `dispatch-trace` forwards Hyperreal and
  Hyperlimit dispatch instrumentation.
- Requires Rust 1.95.0 or later.
