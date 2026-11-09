# HyperBRep IO

HyperBRep IO is the pure Rust STEP interchange boundary for the Hyper workspace.
It owns bounded Part 21 syntax, source-entity accounting, exact unit conversion
and selected representation mapping. [HyperBRep](../hyperbrep/README.md) owns
geometry and topology; Hyperreal owns exact scalar arithmetic.

## Architecture and scope

```text
STEP bytes ── explicit read limits ──► Part 21 Exchange
                                            │
                                  StepFile + representation selection
                                            │
                                  validated native HyperBRep bodies
                                            │
                              exact selected STEP writers + source maps
```

The [mapping contract](docs/STEP_MAPPING.md) is the current authority for
accepted entities, supported families, units, limits and open work. Selected
faceted and advanced representations retain holes, directed edges and source
parameter maps. Supported analytic and spline mappings validate complete
edge/pcurve/surface images; unsupported content is reported explicitly.
Whole-document/product traversal, assembly import and general surface mappings
remain unfinished. Syntax success alone grants no geometric authority.

## Read bounded syntax

`part21::parse` takes explicit `ReadLimits`. Numeric lexemes and unknown records
remain addressable without materializing scalars. `NumberLimits` separately
bounds number materialization, which preserves admitted rational values exactly.
These limits do not bound all later scalar or geometric work.

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

## Import a selected representation

`StepFile::read` adds header, text and unit interpretation. The caller selects a
representation; unselected entities remain accounted for in the source exchange.
Import uses the ordinary native model builder and strict validation, without
healing, welding, fitting or applying uncertainty values as tolerances.

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

Successful import certifies the selected native bodies within the declared
mapping domain. It does not establish complete EXPRESS/AP conformance or
original authoring fidelity. Native model archives and STEP syntax serialization
are separate operations.

## Write selected native geometry

`write_faceted` emits the supported planar faceted subset, including face holes
and multiple selected body occurrences. The report maps native topology to output
entities. Native face-local history and pcurves remain in the original archive.

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

`write_advanced` retains the supported explicit edge identities, curve intervals
and analytic/spline carriers. The report includes source-to-output chart maps.

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

Both writers preserve exact representability. They reject unsupported coordinate
encodings such as `1/3`, unmaterialized selected vertices, unsupported carriers
and cavity shells with typed errors; they never round or drop them. Output unit
selection can itself make coordinates unrepresentable. Native exact support
does not imply STEP encodability, and validating a reimport does not certify the
fidelity of an earlier demotion.

## Build and examples

Run from the CSGRS monorepo root:

```sh
cargo test --locked -p hyperbrep-io --all-features
cargo run -p hyperbrep-io --example region_extrusion_step --release -- /tmp/region-step
```

The [region extrusion example](examples/region_extrusion_step.rs) consumes
native line/circle regions through exchange and later operations. Other runnable
examples cover [faceted output](examples/faceted_step.rs),
[advanced planar import](examples/advanced_step.rs),
[analytic export](examples/analytic_step.rs),
[supplied pcurves](examples/supplied_step.rs), [tori](examples/torus_step.rs)
and [cones](examples/cone_step.rs). Each retains its declared mapping limits.

[Parser review](docs/PARSER_REVIEW.md) records dependency choices;
[performance notes](PERFORMANCE.md) retain measured costs and their scope.
Extensions follow the stack [admission protocol](../../docs/admission.md).
Earlier certification and migration narratives are preserved in the
[historical README snapshot](docs/history/README-2026-11-09.md). Current mapping
and implementation status remains in the existing mapping contract and native
support records, rather than a parallel README ledger.
