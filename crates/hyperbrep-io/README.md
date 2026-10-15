# HyperBRep IO

Pure Rust CAD exchange for the Hyper geometry stack. This crate owns syntax,
source-entity accounting and exchange policy; authoritative geometry belongs to
HyperBRep and exact scalar arithmetic belongs to Hyperreal.

The crate reads and writes bounded Part 21 syntax and imports an explicitly
selected faceted representation into strictly validated native HyperBRep solids.
Coordinates and authored unit conversions remain exact. Whole-document/product
traversal, analytic/spline exchange and STEP geometry export remain unfinished.

The syntax layer retains numeric lexemes without binary floating-point conversion, unknown
entity records, complex instances, forward references, multiple DATA sections,
empty aggregates, typed parameters and encoded string tokens. Every view is
scoped to its source exchange. Nested values use flat storage and explicit
stacks for parsing and writing; cloning and dropping do not recurse through a
parameter tree.

```rust
use hyperbrep_io::part21::{parse, NumberLimits, ReadLimits, ValueKind};
use hyperreal::Real;

let input = b"ISO-10303-21;HEADER;FILE_DESCRIPTION((),'2;1');FILE_NAME('','',(),(),'','','');FILE_SCHEMA(('TEST'));ENDSEC;DATA;#1=POINT(9007199254740993.0);ENDSEC;END-ISO-10303-21;";
let exchange = parse(input, &ReadLimits::default())?;
let component = exchange.entity(1).unwrap().components().next().unwrap();
if let ValueKind::Real(number) = component.arguments().next().unwrap().kind() {
    assert_eq!(number.to_real(&NumberLimits::default())?,
               Real::from(9_007_199_254_740_993_u64));
}
assert_eq!(exchange.unresolved_references().count(), 0);
let output = exchange.to_step(4096)?;
assert!(output.contains("9007199254740993.0"));
# Ok::<(), hyperbrep_io::part21::Error>(())
```

`part21::parse` borrows UTF-8/ASCII input and requires explicit `ReadLimits`.
Limits cover bytes, tokens, token length, nesting, values, records, instances and
DATA sections. Dynamic parser/writer collections reserve fallibly. Number
materialization separately requires digit/exponent limits and produces exact
rational `Real` values. These counts do not bound all work or allocation inside
Hyperreal. Syntax parsing does not materialize numbers.

`Exchange::unresolved_references` reports every missing target with its source
entity and byte span. Cycles are retained: whether a cycle is legal depends on
the later entity schema. Duplicate IDs fail, including IDs with different leading
zeros. IDs must be nonzero. Lookup uses a sorted compact index, never storage
sized by the largest ID. Binary tokens retain their encoding and require valid
leading zero padding.

`Exchange::to_step` requires an output byte allowance. It preserves primitive
lexemes, record/component/section order and unknown records while normalizing
trivia. Encoded strings remain quoted tokens: apostrophes and Unicode/code-page
control directives are preserved, not decoded or semantically validated. This
allows subsequent text decoding to report unsupported/invalid directives without
losing their source. Numeric lexemes such as `1.E400` survive syntax round trips.

The typed `step` layer adds structurally checked standard headers, decoded text,
unit-graph resolution and selected `FACETED_BREP_SHAPE_REPRESENTATION` import.
It preserves all bodies and face holes in that selection, shares exactly equal
vertices/edges, retains directed-segment provenance, and uses the ordinary native
model builder and validator. It neither applies source uncertainty as a tolerance
nor heals orientation, near-coincidence or nonplanarity.

```rust
use hyperbrep::{OperationContext, WorkBudget};
use hyperbrep_io::step::{ImportLimits, StepFile, StepReadOptions};

let input = include_bytes!("../tests/fixtures/faceted-through-hole.step");
let file = StepFile::read(input, &StepReadOptions::default())?;
let context = OperationContext::strict(WorkBudget {
    refinements: 10_000,
    promotions: 1_000,
});
let attempt = file.import_faceted_representation(502, &ImportLimits::default(), &context);
let imported = attempt.result??; // Solver exhaustion and mapping errors stay distinct.
assert_eq!(imported.bodies().len(), 1);
assert_eq!(imported.bodies()[0].model().counts().faces, 10);
assert!(imported.retained_entities().contains(&9503)); // Product remains in the source.
# Ok::<(), Box<dyn std::error::Error>>(())
```

Successful import certifies the selected native bodies within HyperBRep's current
validation domain. It does not establish complete EXPRESS/AP conformance or
import an assembly. The [mapping contract and frontier](docs/STEP_MAPPING.md)
list accepted entities, exact unit conventions, budgets and remaining work.
Unselected/unsupported records remain in the source exchange, with every entity
listed as interpreted or retained. `Model::to_json` is native archival replay;
`Exchange::to_step` is syntax serialization. Neither exports edited geometry to STEP.

ANCHOR, REFERENCE,
SIGNATURE sections, scoped/value/constant references return explicit blockers.
Schema-independent syntax success grants no geometric authority.

[Dependency review](docs/PARSER_REVIEW.md) records why Ruststep and `step-p21`
were not adopted for this boundary. [Performance](PERFORMANCE.md) records measured
costs and the limits of the evidence. Extensions follow the stack's
[admission protocol](../../docs/admission.md).
