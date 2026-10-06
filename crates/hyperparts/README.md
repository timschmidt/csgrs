# Hyperparts

Source-attributed part knowledge graphs and cross-domain handoff records for
the Hyper stack.

Hyperparts records part families and variants, assertions, terminals,
interfaces, compatibility, geometry and physics handles, EDA intake,
procurement facts, and manufacturing capabilities without erasing provenance
or uncertainty. It indexes facts; it does not replace CAD, circuit, physics,
routing, DRC, or process-planning engines.

This README describes crate version `0.3.0`.

## Primary types

| Type | Role |
| --- | --- |
| `PartGraph`, `PartFamily`, `PartVariant` | Queryable knowledge graph |
| `PartId`, `VariantId`, `TerminalId` | Validated stable identities |
| `SourceRef`, `SourceRevision`, `Assertion<T>` | Attributed and revision-aware facts |
| `AssertionValue`, `PartAssertion` | Exact scalar, interval, unknown, conflict, and domain assertions |
| `Terminal`, `Interface`, `Port`, `Pin`, `Pad`, `Lead`, `Hole` | Cross-domain connection surfaces |
| `GeometryHandoffReport`, `PhysicsHandoffReport` | Explicit downstream readiness |
| `PartQuery` and specialized query types | Evidence-preserving discovery requests |
| `EdaAuthoringBundle`, `EdaAuthoringImportResult` | Circuit-JSON-like EDA intake and handoffs |

## Install

```toml
[dependencies]
hyperparts = "0.3.0"
```

There are no default features. `dispatch-trace` forwards Hyperreal’s exact
dispatch instrumentation.

## Quick start

This checked example stores a sourced datasheet assertion, inserts a variant,
and queries the resulting graph.

<!-- quickstart:start -->
```rust
use hyperparts::{
    AssertionConfidence, AssertionValue, GeneralPartAssertion, PartAssertion, PartConstraint,
    PartFamily, PartGraph, PartId, PartQuery, PartVariant, SourceRef, VariantId,
};
use hyperreal::Real;

fn main() -> hyperparts::PartsResult<()> {
    let source = SourceRef::new("datasheet", "rev-a-page-4")?;
    let voltage = GeneralPartAssertion {
        key: "nominal-output-voltage".into(),
        value: AssertionValue::exact_scalar(Real::from(5)),
        unit: Some("V".into()),
        conditions: Vec::new(),
        confidence: AssertionConfidence::Reviewed,
        source,
        revision: None,
    };

    let mut variant = PartVariant::new(VariantId::new("regulator-5v")?, None);
    variant.add_assertion(PartAssertion::General(Box::new(voltage)));

    let mut family = PartFamily::new(PartId::new("regulator")?, "linear regulator");
    family.insert_variant(variant);

    let mut graph = PartGraph::default();
    graph.insert_family(family);
    let result = graph.query_parts(&PartQuery {
        constraints: vec![PartConstraint::PartIdContains("regulator".into())],
    });

    assert_eq!(result.candidates.len(), 1);
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

## Knowledge and ownership model

```text
source + revision + confidence
              │
           assertion
              │
 PartVariant ── terminals / aspects / interfaces / processes
              │
          PartFamily
              │
           PartGraph
       query / compatibility / handoff
```

Large geometry, circuit, physics, routing, and fabrication artifacts stay in
their owning crates and are referenced by stable handles. Missing fields remain
unknown; generated or imported artifacts do not become exact source geometry.

## API guide

### Identity, assertions, and graph construction

- `PartId::new`, `VariantId::new`, `RevisionId::new`, `TerminalId::new`,
  `ManufacturerPartNumber::new`, `InternalPartNumber::new`, and
  `SupplierSku::new` create nonempty typed identities.
- `SourceRef::new` records authority and locator;
  `SourceRevision::new` adds revision/date context.
- `Assertion::{known, unknown, status, value, source}` retains typed facts.
- `AssertionValue::{exact_scalar, interval}` stores exact numeric metadata;
  intervals validate endpoint ordering.
- `PartVariant::{new, add_assertion, add_aspect, add_terminal, add_interface,
  add_subpart, add_process}` builds one revision/variant record.
- `PartFamily::{new, insert_variant, variant, variants}` groups variants.
- `PartGraph::{insert_family, family, add_compatibility, add_relationship,
  add_capability, add_manufacturing_route, add_tool, add_import_report}`
  assembles the graph.

`ComplianceClaim` carries source plus revision or consolidation date so mutable
RoHS/REACH evidence is not treated as timeless.

### Interfaces, electronics, and compatibility

- `PartAspect::new`, `Terminal::new`, and `Interface::new` describe named
  electrical, mechanical, fluidic, optical, assembly, and material surfaces.
- `VoltageEnvelope::new` and `VoltageRange::new` validate exact voltage ranges;
  `overlaps` preserves uncertain endpoint ordering.
- `ElectronicPart`, `ElectronicPackage`, `Pinout`, `PinMap`, `DiePort`,
  `DieNet`, `PowerDomain`, `SupplyRail`, `GroundReference`, ratings, and power
  intent retain electronic structure.
- `PartGraph::safe_connection` returns `SafeConnectionReport`: power-to-ground
  is unsafe, known power envelopes must overlap, and missing polarity or
  voltage facts yield `ConnectionDecision::Unknown`.
- `CompatibilityRelation`, `Relationship`, and their kind/class enums retain
  authored compatibility separately from calculated electrical safety.

### Geometry, physics, and manufacturing handoffs

- `GeometryHandle`, `FootprintHandle`, `PackageBodyHandle`,
  `CurveProfileHandle`, `ModelArtifact`, `GridSystem`, `GridFeature`, and
  `MountingPattern` reference owned geometry and source grids.
- `GeometryHandoffReport` retains `GeometryStatus` and required handles.
- Physical property, material requirement, environmental envelope, thermal
  path, mechanical load path, and mass-property need records feed
  `PhysicsHandoffReport`; `is_certified_ready` checks its retained evidence.
- `Process`, `Operation`, `Capability`, `ToolPart`, `ToolCapability`,
  `ProcessCapability`, and `ManufacturingRoute` describe capability evidence.
- `PartGraph::{capabilities_for_tool, capabilities_for_target,
  has_capability_target}` provides direct capability lookup without claiming
  toolpath or schedule validation.

### Queries

- `PartGraph::query_parts` evaluates `PartQuery` constraints and returns ranked
  candidates, evidence, conflicts, and unknowns.
- `InterfaceQuery`, `TerminalQuery`, `CompatibilityQuery`,
  `CapabilityQuery`, `GeometryQuery`, `ElectricalQuery`, `PhysicsQuery`, and
  `ProcurementQuery` provide domain-specific request carriers.
- `QueryResult<T>` separates candidates, `QueryEvidence`, and `QueryUnknown`;
  `QueryMatchStatus` distinguishes exact match, conflict, and incomplete facts.

The graph favors typed in-memory records. Callers may build external indices or
persist records without discarding source evidence.

### EDA intake

- `EdaExactField` preserves the original decimal spelling and validates its
  conversion to `hyperreal::Real`.
- `EdaAuthoringBundle` accepts Circuit-JSON-like source records, footprint
  expressions, package pins, generated-model references, autorouter output,
  and fabrication output.
- `import_eda_authoring_bundle` returns `EdaAuthoringImportResult` containing a
  populated graph plus circuit residual, route geometry, DRC/fabrication, model,
  and exact-field status.
- `EdaAuthoringImportResult::is_exact_ready` requires every relevant retained
  handoff; missing, rejected, lossy, and review-only fields remain visible.

## Guarantees and boundaries

- Exact numeric metadata uses `hyperreal::Real`.
- Sources, revisions, conditions, confidence, lifecycle, conflicts, and
  explicit unknowns remain first-class.
- Compatibility claims and calculated safety reports are not conflated.
- Procurement and regulatory assertions are time/source dependent.
- Stable handles prevent cross-domain artifacts from being duplicated into the
  knowledge graph.
- Hyperparts delegates geometric evaluation, circuit solution, physical
  simulation, routing, DRC, and manufacturability decisions to their owners.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Hyperreal exact-dispatch instrumentation |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --benches --all-features
```

The query benchmark and reference audit are in
[PERFORMANCE.md](PERFORMANCE.md). EDA intake fuzz ownership is documented in
[fuzz/README.md](fuzz/README.md).

## References

- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).
- Nagel, L. W., and Pederson, D. O. *SPICE (Simulation Program with Integrated
  Circuit Emphasis)*. UCB/ERL M382, 1973.
  [Berkeley report](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1973/22871.html).
- tscircuit. “How tscircuit Works: Compiling Functional React Code to Circuit
  JSON.” [Project article](https://blog.tscircuit.com/p/how-tscircuit-works-compiling-functional).
- KiCad Project. *S-expression File Format*.
  [Official developer documentation](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/index.html).
- LibrePCB. *Developer and File-Format Documentation*.
  [Official documentation](https://developers.librepcb.org/).
- European Union. *Directive 2011/65/EU (RoHS)*.
  [EUR-Lex](https://eur-lex.europa.eu/eli/dir/2011/65/oj).
- European Union. *Regulation (EC) No 1907/2006 (REACH)*.
  [EUR-Lex](https://eur-lex.europa.eu/eli/reg/2006/1907/oj).

## Acknowledgements

Hyperparts builds directly on
[Hyperreal](https://github.com/timschmidt/hyperreal). Geometry, physics,
circuit, path, and DRC handoffs are interpreted by their owning Hyper crates.
The format and regulatory sources above remain authoritative for their
respective external semantics.

## License and contributing

Licensed under Apache-2.0 as declared in [Cargo.toml](Cargo.toml).

Bug reports should include the smallest graph/import bundle, source records,
query or handoff, enabled features, and complete evidence. Before proposing a
change, run formatting, focused tests, all targets/features, and strict Clippy.
