//! Performance atlas for Hyperbrep-IO: bounded ISO 10303-21 syntax, exact
//! STEP import, and exact faceted and advanced STEP export, across file size,
//! model complexity, nesting depth, numeric lexeme size and unit-chain depth.
//!
//! Axes:
//! - file size (KiB) of synthetic Part 21 exchanges of points, references,
//!   complex instances and long strings;
//! - nesting depth of one aggregate, and digits or exponent of one numeric
//!   lexeme;
//! - model complexity of native Hyperbrep solids: profile vertices of a
//!   convex or comb (concave) prism, holes in a machined plate, bodies in an
//!   assembly, and steps of a revolved shaft;
//! - decimal digits of every coordinate, including non-terminating values
//!   that the exact writer refuses;
//! - depth of a conversion-based unit chain, with unit and inch factors.
//!
//! Write operations serialize from a native model built outside the timer.
//! Read operations start from STEP text written outside the timer by the same
//! crate. Round trips do both and compare topology counts.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperbrep-io --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
//! With `--resources`, written files report their entity count; imported
//! models report their vertices, edges and faces as `nodes` and their largest
//! vertex coordinate in bits; traversed exchanges report their value-nesting
//! depth; unit chains report the bits of the resolved exact scale.
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt::Write as _;
use std::rc::Rc;

use hyperbrep::{BodySet, OperationContext, Point2, Point3, Real, WorkBudget, builder};
use hyperbrep_io::part21::{self, ReadLimits, Value, ValueKind};
use hyperbrep_io::step::{
    ImportLimits, ImportedRepresentation, StepBody, StepFile, StepReadOptions, StepWriteOptions,
    WriteLimits, write_advanced, write_faceted,
};
use perf_atlas::{Atlas, Rng, Shape, Shaped, powers_of_two};

#[path = "../tests/support/mod.rs"]
mod support;

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

/// Syntax limits wide enough that the swept sizes, not admission, bound work.
const SYNTAX: ReadLimits = ReadLimits {
    input_bytes: 1 << 30,
    tokens: usize::MAX,
    token_bytes: 1 << 28,
    nesting_depth: 1 << 20,
    values: usize::MAX,
    records: usize::MAX,
    entities: usize::MAX,
    sections: 256,
};

fn read_options() -> StepReadOptions {
    StepReadOptions {
        syntax: SYNTAX,
        header_text_bytes: 1 << 28,
    }
}

fn import_limits() -> ImportLimits {
    ImportLimits {
        entity_visits: usize::MAX,
        scalars: usize::MAX,
        text_bytes: usize::MAX,
        edge_uses: usize::MAX,
        spline_knots: usize::MAX,
        ..ImportLimits::default()
    }
}

fn write_options() -> StepWriteOptions<'static> {
    StepWriteOptions {
        limits: WriteLimits {
            output_bytes: 1 << 30,
            entities: usize::MAX,
            edge_uses: usize::MAX,
            ..WriteLimits::default()
        },
        ..StepWriteOptions::new("atlas.step", "2026-11-27T00:00:00")
    }
}

fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000_000,
        promotions: 1_000_000,
    })
}

/// `n / 1000` metres, a terminating decimal.
fn mm(n: i64) -> Real {
    (Real::from(n) / Real::from(1000)).expect("nonzero divisor")
}

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(mm(x), mm(y))
}

/// `n` vertices on the convex parabola `y = x^2`, closed by its chord.
fn convex_profile(n: usize) -> Vec<Point2> {
    let n = n.max(3) as i64;
    (0..n)
        .map(|i| point(i - n / 2, (i - n / 2).pow(2)))
        .collect()
}

/// A comb with `teeth` rectangular teeth: `4 * teeth + 2` vertices, concave.
fn comb_profile(teeth: usize) -> Vec<Point2> {
    let teeth = teeth.max(1) as i64;
    let mut out = vec![point(0, 0), point(2 * teeth, 0)];
    for t in (0..teeth).rev() {
        out.extend([
            point(2 * t + 2, 5),
            point(2 * t + 1, 5),
            point(2 * t + 1, 2),
            point(2 * t, 2),
        ]);
    }
    out
}

/// A square plate with a grid of `holes` square through-holes.
fn plate(holes: usize) -> Option<BodySet> {
    let side = (holes as f64).sqrt().ceil().max(1.0) as i64;
    let width = 3 * side + 1;
    let outer = [
        point(0, 0),
        point(width, 0),
        point(width, width),
        point(0, width),
    ];
    let holes: Vec<Vec<Point2>> = (0..holes as i64)
        .map(|h| {
            let (x, y) = (1 + 3 * (h % side), 1 + 3 * (h / side));
            vec![
                point(x, y),
                point(x + 2, y),
                point(x + 2, y + 2),
                point(x, y + 2),
            ]
        })
        .collect();
    builder::extrude_region(&outer, &holes, Real::zero(), mm(5)).ok()
}

/// A turned shaft: a radius/axial staircase profile with `steps` shoulders.
fn shaft(steps: usize) -> Option<BodySet> {
    let steps = steps.max(1) as i64;
    let mut profile = vec![point(1, 0)];
    for s in 0..steps {
        let radius = 2 + (steps - s);
        profile.push(point(radius, 3 * s));
        profile.push(point(radius, 3 * s + 3));
    }
    profile.push(point(1, 3 * steps));
    builder::revolve(&profile).ok()
}

fn prism(kind: &str, n: usize) -> Option<BodySet> {
    let profile = match kind {
        "convex prism" => convex_profile(n),
        _ => comb_profile(n.div_ceil(4)),
    };
    builder::extrude(&profile, Real::zero(), mm(7)).ok()
}

/// `count` disjoint cuboids, each its own model.
fn assembly(count: usize, cylinders: bool) -> Option<Vec<BodySet>> {
    (0..count as i64)
        .map(|i| {
            let body = if cylinders {
                builder::cylinder(mm(2), mm(5)).ok()?
            } else {
                builder::cuboid(Point3::origin(), Point3::new(mm(1 + i), mm(2), mm(3))).ok()?
            };
            Some(body)
        })
        .collect()
}

fn step_bodies<'a>(sets: impl IntoIterator<Item = &'a BodySet>) -> Vec<StepBody<'a>> {
    sets.into_iter()
        .flat_map(|set| {
            set.bodies().map(|solid| StepBody {
                solid,
                name: "body",
            })
        })
        .collect()
}

/// Entity definitions in a written exchange, as `nodes`.
fn written(text: Result<String, String>) -> Shaped<Result<String, String>> {
    Shaped::new(text, |text| {
        text.as_ref().ok().map(|text| Shape {
            nodes: text.lines().filter(|line| line.starts_with('#')).count() as u64,
            depth: 0,
            bits: 0,
        })
    })
}

fn faceted(bodies: &[StepBody<'_>]) -> Result<(String, u64), String> {
    let output = write_faceted(bodies, &write_options(), &context())
        .result
        .map_err(|e| format!("{e:?}"))?
        .map_err(|e| format!("{:?}", e.kind))?;
    let representation = output.representation();
    Ok((output.into_text(), representation))
}

fn advanced(bodies: &[StepBody<'_>]) -> Result<(String, u64), String> {
    let output = write_advanced(bodies, &write_options(), &context())
        .result
        .map_err(|e| format!("{e:?}"))?
        .map_err(|e| format!("{:?}", e.kind))?;
    let representation = output.representation();
    Ok((output.into_text(), representation))
}

/// Write with the named writer, keeping only the text.
fn write_text(writer: &str, bodies: &[StepBody<'_>]) -> Shaped<Result<String, String>> {
    let output = if writer == "advanced" {
        advanced(bodies)
    } else {
        faceted(bodies)
    };
    written(output.map(|(text, _)| text))
}

fn import(text: &str, representation: u64) -> Result<ImportedRepresentation, String> {
    let file = StepFile::read(text.as_bytes(), &read_options()).map_err(|e| format!("{e:?}"))?;
    file.import_representation(representation, &import_limits(), &context())
        .result
        .map_err(|e| format!("{e:?}"))?
        .map_err(|e| format!("{:?}", e.kind))
}

fn bits(value: &Real) -> u64 {
    value.representation_size(1 << 16).max_rational_bits
}

/// Imported topology (vertices + edges + faces) and the largest coordinate.
fn imported(
    result: Result<ImportedRepresentation, String>,
) -> Shaped<Result<ImportedRepresentation, String>> {
    Shaped::new(result, |result| {
        let representation = result.as_ref().ok()?;
        let mut shape = Shape::default();
        for body in representation.bodies() {
            let counts = body.model().counts();
            shape.nodes += (counts.vertices + counts.edges + counts.faces) as u64;
            for vertex in body.vertices() {
                if let Some(p) = vertex.vertex.point().try_coordinates() {
                    shape.bits = shape.bits.max(bits(&p.x).max(bits(&p.y)).max(bits(&p.z)));
                }
            }
        }
        Some(shape)
    })
}

type Models = Option<Rc<Vec<BodySet>>>;
type Text = Option<Rc<(String, u64)>>;

thread_local! {
    // Native construction is superlinear in the swept sizes (a 1024-vertex
    // prism takes seconds), so each model and its STEP text are built once
    // and shared by every operation that sweeps them.
    static MODELS: RefCell<HashMap<(&'static str, usize), Models>> = RefCell::default();
    static TEXTS: RefCell<HashMap<(&'static str, usize), Text>> = RefCell::default();
}

/// A model for one complexity kind, or `None` if construction failed.
fn model(kind: &'static str, n: usize) -> Models {
    if let Some(models) = MODELS.with(|cache| cache.borrow().get(&(kind, n)).cloned()) {
        return models;
    }
    let models = build(kind, n).map(Rc::new);
    MODELS.with(|cache| cache.borrow_mut().insert((kind, n), models.clone()));
    models
}

fn build(kind: &str, n: usize) -> Option<Vec<BodySet>> {
    match kind {
        "convex prism" | "comb prism" => prism(kind, n).map(|b| vec![b]),
        "machined plate" => plate(n).map(|b| vec![b]),
        "cuboid assembly" => assembly(n, false),
        "cylinder assembly" => assembly(n, true),
        "turned shaft" => shaft(n).map(|b| vec![b]),
        other => unreachable!("unknown model kind {other}"),
    }
}

/// The writer that exports a kind: curved kinds need the advanced writer.
fn writer_for(kind: &str) -> &'static str {
    match kind {
        "cylinder assembly" | "turned shaft" => "advanced",
        _ => "faceted",
    }
}

/// STEP text and representation ID for a kind, written outside the timer.
fn step_text(kind: &'static str, n: usize) -> Text {
    if let Some(text) = TEXTS.with(|cache| cache.borrow().get(&(kind, n)).cloned()) {
        return text;
    }
    let sets = model(kind, n)?;
    let bodies = step_bodies(sets.iter());
    let result = if writer_for(kind) == "advanced" {
        advanced(&bodies)
    } else {
        faceted(&bodies)
    };
    let text = result.ok().map(Rc::new);
    TEXTS.with(|cache| cache.borrow_mut().insert((kind, n), text.clone()));
    text
}

/// One synthetic DATA entity of the given kind with ID `id`.
fn entity(kind: &str, id: usize, rng: &mut Rng, out: &mut String) {
    let coordinate = |rng: &mut Rng| format!("{}.{:03}", rng.range(-999, 999), rng.below(1000));
    match kind {
        "points" => {
            let (x, y, z) = (coordinate(rng), coordinate(rng), coordinate(rng));
            let _ = writeln!(out, "#{id}=CARTESIAN_POINT('',({x},{y},{z}));");
        }
        "references" => {
            let pick = |rng: &mut Rng| 1 + rng.below(id as u64) as usize;
            let (a, b, c) = (pick(rng), pick(rng), pick(rng));
            let _ = writeln!(out, "#{id}=EDGE_CURVE('',#{a},#{b},#{c},.T.);");
        }
        "complex" => {
            let _ = writeln!(
                out,
                "#{id}=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));"
            );
        }
        _ => {
            let name: String = (0..48)
                .map(|_| char::from(b'a' + rng.below(26) as u8))
                .collect();
            let _ = writeln!(
                out,
                "#{id}=PRODUCT('{name}','it''s part {id}','\\X2\\00E9\\X0\\',(#1));"
            );
        }
    }
}

fn envelope(data: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('atlas'),'2;1');\nFILE_NAME('atlas.step','2026-11-27T00:00:00',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n{data}ENDSEC;\nEND-ISO-10303-21;\n"
    )
}

/// A synthetic exchange of roughly `kib` KiB of entities of one kind.
fn synthetic(kind: &str, kib: usize, rng: &mut Rng) -> String {
    let mut data = String::new();
    let mut id = 1;
    while data.len() < kib * 1024 {
        entity(kind, id, rng, &mut data);
        id += 1;
    }
    envelope(&data)
}

/// Deepest value nesting below `value`, iterative so deep input cannot
/// overflow the traversal itself.
fn depth(value: Value<'_, '_>) -> u64 {
    let mut deepest = 0;
    let mut stack = vec![(value, 1u64)];
    while let Some((value, level)) = stack.pop() {
        deepest = deepest.max(level);
        stack.extend(value.children().map(|child| (child, level + 1)));
    }
    deepest
}

/// Parse, then visit every value of every entity (what an importer does).
fn traverse(text: &str) -> Result<(usize, u64), String> {
    let exchange = part21::parse(text.as_bytes(), &SYNTAX).map_err(|e| format!("{e:?}"))?;
    let mut deepest = 0;
    for entity in exchange.entities() {
        for component in entity.components() {
            for argument in component.arguments() {
                deepest = deepest.max(depth(argument));
            }
        }
    }
    Ok((exchange.entities().len(), deepest))
}

fn traversed(result: Result<(usize, u64), String>) -> Shaped<Result<(usize, u64), String>> {
    Shaped::new(result, |result| {
        result.as_ref().ok().map(|&(nodes, depth)| Shape {
            nodes: nodes as u64,
            depth,
            bits: 0,
        })
    })
}

/// Parse and count entities.
fn parse(text: &str) -> Shaped<Result<usize, String>> {
    let result = part21::parse(text.as_bytes(), &SYNTAX)
        .map(|exchange| exchange.entities().len())
        .map_err(|e| format!("{e:?}"));
    Shaped::new(result, |result| {
        result.as_ref().ok().map(|&nodes| Shape {
            nodes: nodes as u64,
            depth: 0,
            bits: 0,
        })
    })
}

/// Parse, then materialize every numeric lexeme as an exact rational.
fn materialize(text: &str) -> Result<u64, String> {
    let exchange = part21::parse(text.as_bytes(), &SYNTAX).map_err(|e| format!("{e:?}"))?;
    let limits = part21::NumberLimits {
        digits: usize::MAX,
        exponent: usize::MAX,
    };
    let mut largest = 0;
    let mut stack: Vec<Value<'_, '_>> = exchange
        .entities()
        .flat_map(|entity| entity.components())
        .flat_map(|component| component.arguments())
        .collect();
    while let Some(value) = stack.pop() {
        match value.kind() {
            ValueKind::Real(number) | ValueKind::Integer(number) => {
                let real = number.to_real(&limits).map_err(|e| format!("{e:?}"))?;
                largest = largest.max(bits(&real));
            }
            _ => stack.extend(value.children()),
        }
    }
    Ok(largest)
}

const PRISMS: [&str; 2] = ["convex prism", "comb prism"];

fn main() {
    let mut atlas = Atlas::from_args("hyperbrep-io");
    // Inputs are STEP text or immutable native models, and building them
    // (megabytes of text, validated solids) costs far more than one call, so
    // one input serves every call of a point. Models may retain derived facts
    // such as bounds; writers only read them.
    atlas.reuse_inputs(true);
    let kib = powers_of_two(0, 14);

    atlas.operation(
        "part21",
        "parse by file size",
        "file size (KiB)",
        "Bounded Part 21 syntax parse of synthetic DATA sections: Cartesian points, entity references, complex instances, or long quoted strings with escapes.",
    );
    for kind in ["points", "references", "complex", "strings"] {
        atlas.sweep(
            kind,
            &kib,
            |m, rng| synthetic(kind, m as usize, rng),
            |text| parse(text),
        );
    }
    atlas.operation(
        "part21",
        "parse and traverse by file size",
        "file size (KiB)",
        "Parse, then visit every parameter value of every entity, as a typed importer does.",
    );
    for kind in ["points", "references"] {
        atlas.sweep(
            kind,
            &kib,
            |m, rng| synthetic(kind, m as usize, rng),
            |text| traversed(traverse(text)),
        );
    }
    atlas.operation(
        "part21",
        "to_step by file size",
        "file size (KiB)",
        "Canonical re-serialization of a parsed exchange (parsing outside the timer is not possible because the exchange borrows its input, so the call includes it).",
    );
    for kind in ["points", "strings"] {
        atlas.sweep(
            kind,
            &kib,
            |m, rng| synthetic(kind, m as usize, rng),
            |text| {
                let exchange =
                    part21::parse(text.as_bytes(), &SYNTAX).map_err(|e| format!("{e:?}"))?;
                exchange
                    .to_step(1 << 30)
                    .map(|out| out.len())
                    .map_err(|e| format!("{e:?}"))
            },
        );
    }
    atlas.operation(
        "part21",
        "parse by nesting depth",
        "aggregate nesting depth",
        "Parse one entity whose argument is an aggregate nested to the given depth; with traversal, every level is also visited.",
    );
    let depths = powers_of_two(0, 18);
    let nested = |d: f64, _: &mut Rng| {
        let d = d as usize;
        envelope(&format!("#1=A({}1{});\n", "(".repeat(d), ")".repeat(d)))
    };
    atlas.sweep("parse", &depths, nested, |text| parse(text));
    atlas.sweep("parse and traverse", &depths, nested, |text| {
        traversed(traverse(text))
    });
    atlas.operation(
        "part21",
        "materialize number by digits",
        "significand digits",
        "Parse one Cartesian point whose three coordinates have the given number of digits, then materialize each as an exact rational.",
    );
    let digits = powers_of_two(0, 16);
    for (kind, exponent) in [("plain", false), ("with E-exponent", true)] {
        atlas.sweep(
            kind,
            &digits,
            |d, rng| {
                let mut coordinate = || {
                    let digits: String = (0..d as usize)
                        .map(|_| char::from(b'0' + rng.below(10) as u8))
                        .collect();
                    if exponent {
                        format!("1.{digits}E-{}", d as usize)
                    } else {
                        format!("1.{digits}")
                    }
                };
                let (x, y, z) = (coordinate(), coordinate(), coordinate());
                envelope(&format!("#1=CARTESIAN_POINT('',({x},{y},{z}));\n"))
            },
            |text| materialize(text),
        );
    }
    atlas.operation(
        "part21",
        "materialize number by exponent",
        "decimal exponent",
        "Materialize 1.5E+e and 1.5E-e exactly: the stored rational grows with the exponent.",
    );
    let exponents = powers_of_two(0, 14);
    for (kind, sign) in [("positive exponent", '+'), ("negative exponent", '-')] {
        atlas.sweep(
            kind,
            &exponents,
            |e, _| {
                envelope(&format!(
                    "#1=CARTESIAN_POINT('',(1.5E{sign}{},0.,0.));\n",
                    e as usize
                ))
            },
            |text| materialize(text),
        );
    }

    // Native construction outside the timer grows superlinearly (about 6 s
    // for a 1024-vertex prism and 6 s for a 512-hole plate), which bounds
    // these axes; the exchange calls themselves stay far below the ceiling.
    let complexity = powers_of_two(2, 10);
    let counts = powers_of_two(0, 9);
    atlas.operation(
        "step write",
        "write_faceted by profile vertices",
        "profile vertices",
        "Exact faceted (planar polyloop) export of an extruded prism with a convex or comb-shaped (concave) profile.",
    );
    for kind in PRISMS {
        atlas.sweep(
            kind,
            &complexity,
            |n, _| model(kind, n as usize),
            |sets| match sets {
                Some(sets) => write_text("faceted", &step_bodies(sets.iter())),
                None => written(Err("model construction failed".into())),
            },
        );
    }
    atlas.operation(
        "step write",
        "write by feature count",
        "holes, bodies or shoulders",
        "Exact export as one feature count grows: through-holes in a machined plate (faceted), disjoint cuboids in an assembly (faceted), cylinders in an assembly (advanced), or shoulders on a turned shaft (advanced; refused, because the advanced writer does not support revolution surfaces).",
    );
    let features = [
        "machined plate",
        "cuboid assembly",
        "cylinder assembly",
        "turned shaft",
    ];
    // The advanced writer refuses revolution surfaces, so turned shafts are
    // swept (and marked failed) only by the write operation.
    let readable = &features[..3];
    for kind in features {
        atlas.sweep(
            kind,
            &counts,
            |n, _| model(kind, n as usize),
            |sets| match sets {
                Some(sets) => write_text(writer_for(kind), &step_bodies(sets.iter())),
                None => written(Err("model construction failed".into())),
            },
        );
    }
    atlas.operation(
        "step write",
        "write_advanced by profile vertices",
        "profile vertices",
        "Exact advanced (explicit edge/vertex topology) export of the same prisms.",
    );
    for kind in PRISMS {
        atlas.sweep(
            kind,
            &complexity,
            |n, _| model(kind, n as usize),
            |sets| match sets {
                Some(sets) => write_text("advanced", &step_bodies(sets.iter())),
                None => written(Err("model construction failed".into())),
            },
        );
    }
    atlas.operation(
        "step write",
        "write_faceted by coordinate digits",
        "decimal digits per coordinate",
        "Faceted export of a cuboid whose corner coordinates are k / 10^d (terminating), 1 / 2^d (terminating, d digits) or k / (3 * 10^d) (non-terminating, refused).",
    );
    let coordinate_digits = powers_of_two(0, 11);
    for kind in ["k / 10^d", "1 / 2^d", "k / (3 * 10^d)"] {
        atlas.sweep(
            kind,
            &coordinate_digits,
            |d, rng| {
                let d = d as u32;
                let scale = |base: i64, power: u32| {
                    (0..power).fold(Real::one(), |acc, _| acc * Real::from(base))
                };
                let value = |rng: &mut Rng| -> Real {
                    let k = Real::from(rng.range(1, 999_999));
                    let result = match kind {
                        "k / 10^d" => k / scale(10, d),
                        "1 / 2^d" => Real::one() / scale(2, d),
                        _ => k / (scale(10, d) * Real::from(3)),
                    };
                    result.expect("nonzero divisor")
                };
                let max = Point3::new(value(rng), value(rng), value(rng));
                builder::cuboid(Point3::origin(), max).ok()
            },
            |set| match set {
                Some(set) => write_text("faceted", &step_bodies([set])),
                None => written(Err("model construction failed".into())),
            },
        );
    }

    atlas.operation(
        "step read",
        "import by profile vertices",
        "profile vertices",
        "Read a written faceted prism (syntax and header) and import its representation into a validated native solid.",
    );
    for kind in PRISMS {
        atlas.sweep(
            kind,
            &complexity,
            |n, _| step_text(kind, n as usize),
            |text| match text {
                Some(entry) => imported(import(&entry.0, entry.1)),
                None => imported(Err("model construction or write failed".into())),
            },
        );
    }
    atlas.operation(
        "step read",
        "import by feature count",
        "holes, bodies or shoulders",
        "Read and import written plates and assemblies.",
    );
    for &kind in readable {
        atlas.sweep(
            kind,
            &counts,
            |n, _| step_text(kind, n as usize),
            |text| match text {
                Some(entry) => imported(import(&entry.0, entry.1)),
                None => imported(Err("model construction or write failed".into())),
            },
        );
    }
    atlas.operation(
        "step read",
        "read header by profile vertices",
        "profile vertices",
        "Bounded syntax parse plus standard-header decoding of a written faceted prism, without geometry import.",
    );
    for kind in PRISMS {
        atlas.sweep(
            kind,
            &complexity,
            |n, _| step_text(kind, n as usize),
            |text| {
                let entry = text.as_ref().ok_or("model construction or write failed")?;
                StepFile::read(entry.0.as_bytes(), &read_options())
                    .map(|file| file.syntax().entities().len())
                    .map_err(|e| format!("{e:?}"))
            },
        );
    }
    atlas.operation(
        "step read",
        "context_units by chain depth",
        "conversion-based unit chain depth",
        "Resolve a chain of conversion-based length units to an exact SI scale; factor 1 keeps the scale at 1, factor 25.4 grows it (refused past the 65536-bit unit-scale limit).",
    );
    let chains = powers_of_two(0, 14);
    for (kind, factor) in [("factor 1", "1."), ("factor 25.4", "25.4")] {
        atlas.sweep(
            kind,
            &chains,
            |d, _| support::unit_chain(d as usize, factor),
            |text| {
                let units = StepFile::read(text.as_bytes(), &read_options())
                    .map_err(|e| format!("{e:?}"))
                    .and_then(|file| {
                        file.context_units(1, &import_limits())
                            .map_err(|e| format!("{:?}", e.kind))
                    });
                Shaped::new(units, |units| {
                    units.as_ref().ok().map(|units| Shape {
                        nodes: 0,
                        depth: 0,
                        bits: bits(&units.length.scale_to_si),
                    })
                })
            },
        );
    }

    atlas.operation(
        "round trip",
        "write, read and import",
        "profile vertices or features",
        "Write a native model, parse the text, import the representation, and compare face counts with the source; prisms by profile vertices, the rest by feature count.",
    );
    for kind in PRISMS.into_iter().chain(readable.iter().copied()) {
        let axis = if PRISMS.contains(&kind) {
            &complexity
        } else {
            &counts
        };
        atlas.sweep(
            kind,
            axis,
            |n, _| model(kind, n as usize),
            |sets| {
                let Some(sets) = sets else {
                    return imported(Err("model construction failed".into()));
                };
                let bodies = step_bodies(sets.iter());
                let written = if writer_for(kind) == "advanced" {
                    advanced(&bodies)
                } else {
                    faceted(&bodies)
                };
                let result = written.and_then(|(text, representation)| {
                    let imported = import(&text, representation)?;
                    let faces = |counts: hyperbrep::ModelCounts| counts.faces;
                    let source: usize = sets.iter().map(|set| faces(set.model().counts())).sum();
                    let back: usize = imported
                        .bodies()
                        .iter()
                        .map(|body| faces(body.model().counts()))
                        .sum();
                    if source == back {
                        Ok(imported)
                    } else {
                        Err(format!("face count changed: {source} -> {back}"))
                    }
                });
                imported(result)
            },
        );
    }
    atlas.finish();
}
