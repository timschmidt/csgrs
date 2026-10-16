# HyperBRep IO

Pure Rust CAD exchange for the Hyper geometry stack. This crate owns syntax,
source-entity accounting and exchange policy; authoritative geometry belongs to
HyperBRep and exact scalar arithmetic belongs to Hyperreal.

The crate reads and writes bounded Part 21 syntax, imports an explicitly
selected faceted or supported advanced representation into strictly validated
native HyperBRep solids, and exports selected planar/cylindrical solids with
line/circle boundaries to STEP.
Advanced import includes circular edges on planes and cylinders and supplied
planar/cylindrical pcurves. Coordinates
and authored unit conversions remain exact. Whole-document/product traversal,
other analytic/spline mappings, cavity shells and approximate export remain unfinished.

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
unit-graph resolution and selected faceted/advanced representation import.
It preserves all bodies and face holes in that selection and uses the ordinary native
model builder and validator. It neither applies source uncertainty as a tolerance
nor heals orientation, near-coincidence or nonplanarity.

Faceted polygons share exact rational coordinate aliases. Advanced planar faces
retain explicit `VERTEX_POINT` and `EDGE_CURVE` identity without welding. Their
spatial lines and every supplied planar pcurve must describe the same oriented
line exactly. Different parameterizations are supported: `explicit_edges()`
returns source parameter maps and the declared master representation. Bound
orientation and edge sense remain distinct; winding identifies finite outer
bounds even when the file only contains `FACE_BOUND` records. `VertexSource` and
`EdgeUseSource` distinguish implicit polygons from explicit topology.

Direct `CIRCLE` edges retain the source angular frame, traversal and parameter
map, including a closed edge with one vertex and distinct seam uses. Cylinder
charts use native radians and metres; source angle units are converted exactly
and independently of length units. Native validation proves the complete
edge/pcurve image. Selected `SURFACE_CURVE` records admit planar `CIRCLE` and
cylindrical `LINE` pcurves, with every alternate checked and its parameter map
retained. `SEAM_CURVE` additionally requires two pcurves at opposite exact
parameter bounds on the same surface. Decimal approximations to a full angular
period remain typed rejections; there is no inferred `pi` or implicit repair.
See the [supplied-pcurve example](examples/supplied_step.rs) and
[mapping contract](docs/STEP_MAPPING.md#supplied-analytic-pcurves).

```rust
use hyperbrep::{OperationContext, WorkBudget};
use hyperbrep_io::step::{ImportLimits, StepFile, StepReadOptions};

let input = include_bytes!("../tests/fixtures/faceted-through-hole.step");
let file = StepFile::read(input, &StepReadOptions::default())?;
let context = OperationContext::strict(WorkBudget {
    refinements: 10_000,
    promotions: 1_000,
});
let attempt = file.import_representation(502, &ImportLimits::default(), &context);
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
`Exchange::to_step` is syntax serialization.

`step::write_faceted` exports selected native geometry, including edited edges,
holes, reflected solids and multiple body occurrences. It emits the common AP214
faceted subset and reports every source face, wire, vertex and directed edge use.
Shared points follow native identity within each body. Plane charts are rebuilt
exactly from directed boundaries; original pcurves/history require a native archive.

```rust
use hyperbrep::{builder, OperationContext, Point3, Real, WorkBudget};
use hyperbrep_io::step::{StepBody, StepWriteOptions, write_faceted};

// Native exchange coordinates are metres. An eighth is exactly decimal-representable.
let eighth = (Real::one() / Real::from(8))?;
let bodies = builder::cuboid(Point3::origin(),
    Point3::new(eighth, Real::one(), Real::one()))?;
let input = StepBody { solid: bodies.bodies().next().unwrap(), name: "Block" };
let context = OperationContext::strict(WorkBudget { refinements: 0, promotions: 0 });
let options = StepWriteOptions::new("block.step", "2026-10-15T00:00:00");
let artifact = write_faceted(&[input], &options, &context).result??;
assert_eq!(artifact.bodies()[0].edge_uses().len(), 24);
assert!(artifact.text().contains("FACETED_BREP"));
# Ok::<(), Box<dyn std::error::Error>>(())
```

Both writers reject `1/3`, irrational coordinates, unmaterialized selected points,
unmapped families and cavity shells with typed source-addressable errors. They never
rounds, fits, heals or drops unsupported geometry. `StepWriteOptions` makes units,
metadata and allowances explicit; the caller supplies the timestamp. Exporting
metres as inches can itself make a coordinate nonrepresentable. The
[native export example](examples/faceted_step.rs) writes seven original
construction recipes for independent readers. The
[advanced import example](examples/advanced_step.rs) reads independent planar
STEP bodies, splits and reflects a holed solid, exports faceted STEP, and
checks exact native reimport. This changes the exchange encoding while retaining
the planar line geometry; it is not curved-face tessellation.
The [circular import example](examples/circular_step.rs) imports an exact
non-cardinal cylinder, splits a circular edge and validates native archive replay.
It checks the exact volume and deterministic archive replay independently of
STEP output; the faceted writer rejects its curved carriers.

`step::write_advanced` emits explicit `VERTEX_POINT`/`EDGE_CURVE` identities,
directed uses, bounds and `ADVANCED_FACE` records on native planes and cylinders.
It retains straight lines and circular arcs, including closed circular edges,
shared seams and partial arcs. The borrowed `AdvancedStep` report maps every
used native entity to its output entity and retains exact output curve parameter
intervals and native-to-output surface chart maps. Direct spatial curves are
valid in this selected STEP encoding; native face-local pcurves and history
remain in the original model/archive.

```rust
use hyperbrep::{builder, OperationContext, WorkBudget};
use hyperbrep_io::step::{StepBody, StepWriteOptions, write_advanced};
let native = builder::cylinder(5.into(), 7.into())?;
let context = OperationContext::strict(WorkBudget { refinements: 0, promotions: 0 });
let output = write_advanced(&[StepBody {
    solid: native.bodies().next().unwrap(), name: "Cylinder",
}], &StepWriteOptions::new("cylinder.step", "2026-10-16T00:00:00"), &context).result??;
assert!(output.text().contains("CYLINDRICAL_SURFACE"));
assert!(output.bodies()[0].edges().iter().any(|e| e.source.curve().circular_geometry().is_some()));
# Ok::<(), Box<dyn std::error::Error>>(())
```

The [analytic export example](examples/analytic_step.rs) exercises native/imported
cylinders, radical frames, edge splitting, reflection, exact export and reimport.
The [native similarity example](examples/analytic_similarity.rs) scales a
cylinder exactly, including reflection, before exporting in millimetres or
inches. Native physical scaling and output unit selection are separate operations;
unit selection does not change the source model. The shared input selection is now
`StepBody`; the former `FacetedBody` name was removed without an alias. Existing
faceted encoding and its deterministic output contract remain supported.

ANCHOR, REFERENCE,
SIGNATURE sections, scoped/value/constant references return explicit blockers.
Schema-independent syntax success grants no geometric authority.

[Dependency review](docs/PARSER_REVIEW.md) records why Ruststep and `step-p21`
were not adopted for this boundary. [Performance](PERFORMANCE.md) records measured
costs and the limits of the evidence. Extensions follow the stack's
[admission protocol](../../docs/admission.md).

Repeated native recuts and complementary reconstruction of torus/cone halves
remain outside the current STEP surface mapping. Export of their independently
replayed outputs returns `UnsupportedSurface` with body/entity identity; the same
operation context still exports a supported cylinder afterwards. The
[composition evidence](../hyperbrep/docs/evidence/2026-10-16-curved-coplanar/README.md)
records this downstream boundary separately from native Boolean completion.
