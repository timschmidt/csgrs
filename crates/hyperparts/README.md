# hyperparts

`hyperparts` is a source-attributed part knowledge graph for the Hyper ecosystem. It
records part families, variants, revisions, assertions, terminals, interfaces,
compatibility relations, process capabilities, import reports, and query evidence.

The crate is not a CAD kernel, physics engine, circuit simulator, router, DRC engine, or
fabrication planner. It indexes facts and external handles for those crates while
preserving source, confidence, and uncertainty.

## Hyper Ecosystem

`hyperparts` is the evidence layer for part and process knowledge.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar values in
  source-attributed metadata.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypermesh](https://github.com/timschmidt/hypermesh), and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): geometry or grid artifact
  handles referenced by shape assertions.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): material, mass, thermal,
  load, and environmental property handoffs.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): electrical models,
  packages, pins, ratings, and safe-connection reports.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): routing, manufacturability, and
  release-package checks that consume part/interface facts.

## Typical Part-Library Problems

Part libraries frequently mix verified datasheet values, guessed package metadata, CAD
handles, supplier lifecycle fields, and manufacturing assumptions in one table. Missing
data becomes ambiguous: it may mean safe default, not applicable, proprietary, or simply
not stated by the source. Downstream tools then inherit guesses that look authoritative.

`hyperparts` treats part knowledge as evidence. Assertions carry source references,
confidence, status, and explicit unknowns. Compatibility, safe-connection, geometry,
physics, electronics, and process handoffs report whether a fact is exact, certified,
conditional, lossy, unsupported, or missing.

## Main Types

- `PartFamily`, `PartVariant`, `PartGraph`, and stable ID types describe the knowledge
  graph.
- `SourceRef`, `SourceRevision`, `Assertion<T>`, `AssertionValue`, and `PartAssertion`
  preserve provenance and known/unknown status.
- `Terminal`, `Interface`, `Port`, `Pin`, `Pad`, `Lead`, `Hole`, and
  `ReferenceDesignatorClass` describe external connection and assembly surfaces.
- `GeometryHandoffReport`, `PhysicsHandoffReport`, `ElectricalCompatibilityReport`,
  and `SafeConnectionReport` preserve downstream handoff evidence.
- `ToolPart`, `Process`, `Operation`, `Capability`, `ManufacturingRoute`, and related
  process types describe manufacturing capability without becoming a scheduler.
- `ImportReport`, `PartQueryEvidence`, and `PartQueryResult<T>` make importer and query
  uncertainty explicit.

## Precision Model

Numeric metadata uses `Real` where exact values or ranges are known. Unknown,
conditional, lossy, external, and conflicting facts are represented as data rather than
collapsed into defaults. Source references and assertion status stay attached to values
so downstream crates can choose whether a fact is suitable for exact geometry,
simulation, routing, or readiness checks.

## Performance Model

The crate favors typed records and query evidence over a heavyweight database engine.
Stable IDs make graph edges cheap to compare and serialize. Compatibility and
safe-connection queries return compact reports that can be cached by callers. Import
reports count unknowns and issues at ingestion time so downstream workflows do not have
to rediscover missing evidence repeatedly.

## Current Status

Implemented today:

- stable part, variant, revision, terminal, manufacturer, supplier, and internal IDs;
- source-attributed assertions, values, ranges, lifecycle, compliance, procurement, and
  confidence carriers;
- part graphs with subparts, terminals, interfaces, aspects, and relationships;
- geometry, physics, electronics, EDA, process, tool, operation, fixture, and route
  handoff records;
- compatibility, import, query, and conservative safe-connection reports.

Known limits: `hyperparts` records and connects evidence; it deliberately leaves exact
geometry, circuit solving, physics simulation, routing, and manufacturability decisions
to sibling crates.

## Installation

```toml
[dependencies]
hyperparts = "0.1.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperparts = { path = "../hyperparts" }
```

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench queries
```
