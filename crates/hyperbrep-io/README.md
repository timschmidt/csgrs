# HyperBRep IO

Pure Rust CAD exchange for the Hyper geometry stack. This crate owns syntax,
source-entity accounting and exchange policy; authoritative geometry belongs to
HyperBRep and exact scalar arithmetic belongs to Hyperreal.

The initial implementation reads and writes a bounded Part 21 syntax subset.
It retains numeric lexemes without binary floating-point conversion, unknown
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

This is not yet a STEP-to-BREP importer or AP conformance claim. Header/schema
requirements, EXPRESS types, units, placements, topology, geometry, decoded text,
assemblies and exact/approximated geometry export remain on the
[development plan](../hyperbrep/docs/DEVELOPMENT_PLAN.md). ANCHOR, REFERENCE,
SIGNATURE sections, scoped/value/constant references return explicit blockers.
Schema-independent syntax success grants no geometric authority.

[Dependency review](docs/PARSER_REVIEW.md) records why Ruststep and `step-p21`
were not adopted for this boundary. [Performance](PERFORMANCE.md) records measured
costs and the limits of the evidence. Extensions follow the stack's
[admission protocol](../../docs/admission.md).
