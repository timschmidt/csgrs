<h1>
  hyperparts
</h1>

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
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and transform
  carriers used by geometry and physics siblings.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy used by
  downstream geometry and readiness crates.
- [hypertri](https://github.com/timschmidt/hypertri): triangulation evidence for
  geometry and footprint handoffs.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypermesh](https://github.com/timschmidt/hypermesh), and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): geometry or grid artifact
  handles referenced by shape assertions.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual replay and solver
  evidence referenced by circuit, physics, and process handoffs.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): material, mass, thermal,
  load, and environmental property handoffs.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): electrical models,
  packages, pins, ratings, and safe-connection reports.
- [hyperpath](https://github.com/timschmidt/hyperpath) and
  [hyperdrc](https://github.com/timschmidt/hyperdrc): routing, manufacturability, and
  release-package checks that consume part/interface facts.
- [hyperpack](https://github.com/timschmidt/hyperpack): package, bin, and process
  placement consumers of part evidence.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): search layer for
  candidate part, process, and package selections.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): boundary-representation geometry
  handles for future package/body handoffs.
- [hypersdf](https://github.com/timschmidt/hypersdf): implicit-field and clearance
  evidence for future package, fixture, and process queries.

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
- `GeneralPartAssertion`, `AssertionCondition`, `AssertionConfidence`,
  `ComplianceClaim`, `ProcurementOffer`, and `PartKnowledgeReport` preserve scalar,
  text, range, compliance, sourcing, conflict, and unknown evidence.
- `Terminal`, `Interface`, `Port`, `Pin`, `Pad`, `Lead`, `Hole`, and
  `ReferenceDesignatorClass` describe external connection and assembly surfaces.
- `GeometryHandoffReport`, `PhysicsHandoffReport`, `ElectricalCompatibilityReport`,
  and `SafeConnectionReport` preserve downstream handoff evidence.
- `ToolPart`, `Process`, `Operation`, `Capability`, `ManufacturingRoute`, and related
  process types describe manufacturing capability without becoming a scheduler.
- `ImportReport`, `PartQueryEvidence`, and `PartQueryResult<T>` make importer and query
  uncertainty explicit.
- `PartQuery`, `GeometryQuery`, `ElectricalQuery`, `PhysicsQuery`, `CapabilityQuery`,
  `ProcurementQuery`, `QueryResult<T>`, and `QueryMatchStatus` make discovery and
  selection evidence report-bearing rather than implicit.

## Precision Model

Numeric metadata uses `Real` where exact values or ranges are known. Unknown,
conditional, lossy, external, and conflicting facts are represented as data rather than
collapsed into defaults. Source references and assertion status stay attached to values
so downstream crates can choose whether a fact is suitable for exact geometry,
simulation, routing, or readiness checks.

Numerical explosion is controlled by storing handles, source references, exact scalars,
exact intervals, and explicit unknowns instead of copying downstream geometry, circuit,
physics, or process state into the part graph. A part assertion points to the owning
crate and evidence; it does not become a second unsynchronized model of that domain.

## Performance Model

The crate favors typed records and query evidence over a heavyweight database engine.
Stable IDs make graph edges cheap to compare and serialize. Compatibility and
safe-connection queries return compact reports that can be cached by callers. Import
reports count unknowns and issues at ingestion time so downstream workflows do not have
to rediscover missing evidence repeatedly.

Performance should come from indexing and caching over stable IDs, not from erasing
provenance. Missing fields remain compact unknown records, while large CAD, EDA,
physics, routing, or manufacturing artifacts stay in their owning crates.

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
hyperparts = "0.2.0"
```

For sibling checkouts:

```toml
[dependencies]
hyperparts = { path = "../hyperparts" }
```

## Usage

Represent catalog facts as source-attributed evidence, not anonymous table cells:

```rust,ignore
use hyperparts::{
    AssertionConfidence, AssertionValue, GeneralPartAssertion, PartAssertion, PartFamily,
    PartGraph, PartId, PartQueryEvidence, SourceRef, VariantId, PartVariant,
};
use hyperreal::Real;

let source = SourceRef::new("datasheet", "rev-a-page-4")?;
let voltage = GeneralPartAssertion {
    key: "nominal-output-voltage".into(),
    value: AssertionValue::exact_scalar(Real::from(5)),
    unit: Some("V".into()),
    conditions: Vec::new(),
    confidence: AssertionConfidence::Reviewed,
    source: source.clone(),
    revision: None,
};

let mut variant = PartVariant::new(VariantId::new("regulator-5v")?, None);
variant.add_assertion(PartAssertion::General(Box::new(voltage)));

let mut family = PartFamily::new(PartId::new("regulator")?, "linear regulator");
family.insert_variant(variant);

let mut graph = PartGraph::default();
graph.insert_family(family);

let evidence = PartQueryEvidence::from_fact(source, "datasheet voltage stated");
assert_eq!(evidence.facts.len(), 1);
```

Compatibility queries, safe-connection reports, geometry handoffs, physics handoffs,
and manufacturing-route records use the same source/status pattern so downstream crates
can distinguish certified facts, missing evidence, and lossy imports.

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23.
- Nagel, Laurence W., and Donald O. Pederson. *SPICE (Simulation Program with Integrated
  Circuit Emphasis)*. ERL-M382, University of California, Berkeley, 1973.
- European Union. Directive 2011/65/EU on the restriction of hazardous substances in
  electrical and electronic equipment (RoHS).
- European Union. Regulation (EC) No 1907/2006 concerning Registration, Evaluation,
  Authorisation and Restriction of Chemicals (REACH).
- KiCad Project. *KiCad PCB File Format / S-expression Board Format*.
- LibrePCB Project. *LibrePCB File Format Documentation*.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench queries
```
