# hyperparts

`hyperparts` is a source-attributed part knowledge graph for the Hyper stack.
It records part families and variants, assertions, terminals, interfaces,
compatibility, geometry and physics handles, EDA intake, procurement facts, and
manufacturing capabilities without erasing provenance or uncertainty.

The crate is not a CAD kernel, circuit simulator, physics engine, router, DRC
engine, or process planner. It indexes facts and explicit handoff records for
those domains.

## Quick start

```toml
[dependencies]
hyperparts = "0.3"
hyperreal = "0.13.1"
```

Store a datasheet claim, insert a variant, and query the graph:

```rust,ignore
use hyperparts::{
    AssertionConfidence, AssertionValue, GeneralPartAssertion, PartAssertion,
    PartConstraint, PartFamily, PartGraph, PartId, PartQuery, PartVariant,
    SourceRef, VariantId,
};
use hyperreal::Real;

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

# Ok::<(), hyperparts::PartsError>(())
```

## Core API

- `PartFamily`, `PartVariant`, and `PartGraph` form the queryable graph. Typed
  IDs such as `PartId`, `VariantId`, and `TerminalId` reject empty values.
- `SourceRef`, `SourceRevision`, `Assertion<T>`, `AssertionValue`, and
  `PartAssertion` retain source identity, exact values, intervals, review state,
  explicit unknowns, and conflicts.
- `ComplianceClaim` carries its source and an optional revision or consolidation
  date so mutable RoHS/REACH evidence is not treated as timeless.
- `Terminal`, `Interface`, `Port`, `Pin`, `Pad`, `Lead`, and `Hole` describe
  electrical, mechanical, fluidic, optical, and assembly surfaces.
- `GeometryHandoffReport`, `PhysicsHandoffReport`,
  `ElectricalCompatibilityReport`, and `SafeConnectionReport` keep downstream
  ownership and readiness visible.
- `Process`, `Operation`, `Capability`, `ToolPart`, and `ManufacturingRoute`
  describe process evidence without claiming to schedule or validate toolpaths.
- `PartQuery`, `GeometryQuery`, `ElectricalQuery`, `PhysicsQuery`,
  `CapabilityQuery`, and `ProcurementQuery` describe discovery requests;
  `QueryResult<T>` returns ranked candidates, evidence, conflicts, and unknowns.

`PartGraph::safe_connection` is conservative: a power-to-ground connection is
unsafe, two known power envelopes must overlap, and missing polarity or voltage
facts produce `ConnectionDecision::Unknown` rather than a guessed answer.

## EDA intake

`import_eda_authoring_bundle` ingests Circuit-JSON-like records, exact numeric
strings, footprint expressions, package pins, generated model references,
autorouter output, and fabrication output. It returns both a populated
`PartGraph` and report-bearing handoffs.

`EdaExactField` preserves the original decimal spelling and parses it into
`hyperreal::Real` only after validation. Missing, rejected, lossy, and
review-only fields remain visible in `EdaAuthoringImportResult`; generated
models and routes cannot silently become exact geometry.

## Precision and scaling

Exact numeric metadata uses `Real`; exact ranges validate their ordering.
Unknown, conditional, lossy, external, and conflicting claims are distinct
data states rather than defaults. Large geometry, circuit, physics, routing,
and fabrication artifacts stay in their owning crates and are referenced by
stable handles, limiting duplication and numerical growth.

The in-memory graph favors typed records and stable IDs over a database engine.
Callers can index IDs and cache compact query or compatibility reports without
discarding source evidence.

Current limits are intentional: `hyperparts` connects evidence but delegates
geometric evaluation, circuit solution, physical simulation, routing, and
manufacturability decisions to their domain owners.

## Development

```sh
cargo fmt --all --check
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cargo bench --bench queries
```

The `fuzz/` package contains an `eda_authoring_intake` target for `cargo fuzz`.
See [PERFORMANCE.md](PERFORMANCE.md) for the benchmark and per-reference audit.

## References

- Chee K. Yap, [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2), *Computational Geometry* 7(1–2), 1997.
- Laurence W. Nagel and Donald O. Pederson, [*SPICE (Simulation Program with Integrated Circuit Emphasis)*](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1973/22871.html), UCB/ERL M382, 1973.
- tscircuit, [“How tscircuit Works: Compiling Functional React Code to Circuit JSON”](https://blog.tscircuit.com/p/how-tscircuit-works-compiling-functional).
- KiCad, [S-expression file-format documentation](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/index.html).
- LibrePCB, [developer and file-format documentation](https://developers.librepcb.org/).
- European Union, [Directive 2011/65/EU (RoHS)](https://eur-lex.europa.eu/eli/dir/2011/65/oj) and [Regulation (EC) No 1907/2006 (REACH)](https://eur-lex.europa.eu/eli/reg/2006/1907/oj).

Direct numeric dependency: [hyperreal](https://github.com/timschmidt/hyperreal).
Related domain owners: [hypercurve](https://github.com/timschmidt/hypercurve) ·
[hyperbrep](https://github.com/timschmidt/hyperbrep) ·
[hyperphysics](https://github.com/timschmidt/hyperphysics) ·
[hypercircuit](https://github.com/timschmidt/hypercircuit) ·
[hyperpath](https://github.com/timschmidt/hyperpath) ·
[hyperdrc](https://github.com/timschmidt/hyperdrc).

## License

Apache-2.0.
