//! Exact graph assertions, shared with the real WASM consumer.
use hyperbrep::*;
use hyperbrep_io::{
    part21::{NumberLimits, Value, ValueKind},
    step::{
        AdvancedStep, ImportLimits, ImportedRepresentation, StepBody, StepFile, StepReadOptions,
        StepWriteOptions, write_advanced,
    },
};
use hyperlimit::{PredicateOutcome, compare_reals};
use std::cmp::Ordering;
use std::collections::HashSet;
#[allow(dead_code)]
#[path = "analytic_cylinder.rs"]
pub mod cylinder;
#[allow(dead_code)]
#[path = "supplied_pcurves.rs"]
mod supplied;

pub fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    })
}
pub fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
pub fn equal(a: &Real, b: &Real) {
    assert!(
        matches!(
            compare_reals(a, b, STRICT_PREDICATES),
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                ..
            }
        ),
        "{a:?} != {b:?}"
    );
}
fn equal_point(a: &Point3, b: &Point3) {
    equal(&a.x, &b.x);
    equal(&a.y, &b.y);
    equal(&a.z, &b.z);
}
fn argument<'e, 'a>(file: &'e StepFile<'a>, id: u64, index: usize) -> Value<'e, 'a> {
    file.syntax()
        .entity(id)
        .unwrap()
        .components()
        .next()
        .unwrap()
        .arguments()
        .nth(index)
        .unwrap()
}
fn name<'a>(file: &StepFile<'a>, id: u64) -> &'a str {
    file.syntax()
        .entity(id)
        .unwrap()
        .components()
        .next()
        .unwrap()
        .name()
}
fn reference(value: Value<'_, '_>) -> u64 {
    let ValueKind::Reference(id) = value.kind() else {
        panic!("reference")
    };
    id
}
fn refs(value: Value<'_, '_>) -> Vec<u64> {
    value.children().map(reference).collect()
}
fn scalar(value: Value<'_, '_>) -> Real {
    let ValueKind::Real(number) = value.kind() else {
        panic!("real")
    };
    number.to_real(&NumberLimits::default()).unwrap()
}
fn vector(file: &StepFile<'_>, id: u64) -> Vector3 {
    Vector3::new(
        argument(file, id, 1)
            .children()
            .map(scalar)
            .collect::<Vec<_>>()
            .try_into()
            .unwrap(),
    )
}
fn point(file: &StepFile<'_>, id: u64) -> Point3 {
    Point3::from(vector(file, id))
}
fn frame(file: &StepFile<'_>, id: u64) -> (Point3, Vector3, Vector3, Vector3) {
    assert_eq!(name(file, id), "AXIS2_PLACEMENT_3D");
    let origin = point(file, reference(argument(file, id, 1)));
    let z = vector(file, reference(argument(file, id, 2)))
        .normalize_checked()
        .unwrap();
    let x = vector(file, reference(argument(file, id, 3)))
        .normalize_checked()
        .unwrap();
    equal(&x.dot(&z), &Real::zero());
    let y = z.cross(&x);
    (origin, x, y, z)
}
fn boolean(value: Value<'_, '_>, expected: bool) {
    assert_eq!(
        value.kind(),
        ValueKind::Enumeration(if expected { "T" } else { "F" })
    );
}

/// Independently interprets the selected STEP carrier formulas and all topology.
/// It does not call the geometry importer or use the writer's geometry views.
pub fn graph(output: &AdvancedStep<'_>, scale: &Real) {
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    assert_eq!(file.syntax().unresolved_references().count(), 0);
    assert_eq!(
        name(&file, output.representation()),
        "ADVANCED_BREP_SHAPE_REPRESENTATION"
    );
    assert_eq!(
        refs(argument(&file, output.representation(), 1)),
        output
            .bodies()
            .iter()
            .map(|b| b.entity())
            .collect::<Vec<_>>()
    );
    let mut all_vertices = HashSet::new();
    let mut all_edges = HashSet::new();
    for body in output.bodies() {
        assert_eq!(name(&file, body.entity()), "MANIFOLD_SOLID_BREP");
        assert_eq!(reference(argument(&file, body.entity(), 1)), body.shell());
        assert_eq!(name(&file, body.shell()), "CLOSED_SHELL");
        assert_eq!(
            refs(argument(&file, body.shell(), 1)),
            body.faces().iter().map(|f| f.entity).collect::<Vec<_>>()
        );
        assert_eq!(
            body.faces().iter().map(|f| f.source).collect::<Vec<_>>(),
            body.source().outer().faces().collect::<Vec<_>>()
        );
        let vertex_id = |source| {
            body.vertices()
                .iter()
                .find(|v| v.source == source)
                .unwrap()
                .entity
        };
        let edge_id = |source| {
            body.edges()
                .iter()
                .find(|e| e.source == source)
                .unwrap()
                .entity
        };
        for vertex in body.vertices() {
            assert!(
                all_vertices.insert(vertex.entity),
                "no welding across occurrences"
            );
            assert_eq!(name(&file, vertex.entity), "VERTEX_POINT");
            assert_eq!(reference(argument(&file, vertex.entity, 1)), vertex.point);
            let encoded = Point3::from(vector(&file, vertex.point) * scale);
            equal_point(&encoded, vertex.source.point().try_coordinates().unwrap());
        }
        for edge in body.edges() {
            assert!(
                all_edges.insert(edge.entity),
                "one explicit edge per native identity"
            );
            assert_eq!(name(&file, edge.entity), "EDGE_CURVE");
            assert_eq!(
                reference(argument(&file, edge.entity, 1)),
                vertex_id(edge.source.start())
            );
            assert_eq!(
                reference(argument(&file, edge.entity, 2)),
                vertex_id(edge.source.end())
            );
            assert_eq!(reference(argument(&file, edge.entity, 3)), edge.geometry);
            let increasing = matches!(
                compare_reals(
                    &edge.parameter_start,
                    &edge.parameter_end,
                    STRICT_PREDICATES
                ),
                PredicateOutcome::Decided {
                    value: Ordering::Less,
                    ..
                }
            );
            boolean(argument(&file, edge.entity, 4), increasing);
            let (a, b) = edge.source.domain().try_real_endpoints().unwrap();
            for i in 0..=4 {
                let fraction = ratio(i, 4);
                let parameter = a + (b - a) * &fraction;
                let t = &edge.parameter_start
                    + (&edge.parameter_end - &edge.parameter_start) * fraction;
                let encoded = match name(&file, edge.geometry) {
                    "LINE" => {
                        let origin = point(&file, reference(argument(&file, edge.geometry, 1)));
                        let v = reference(argument(&file, edge.geometry, 2));
                        let direction = vector(&file, reference(argument(&file, v, 1)))
                            .normalize_checked()
                            .unwrap();
                        origin + direction * scalar(argument(&file, v, 2)) * t
                    }
                    "CIRCLE" => {
                        let (center, x, y, _) =
                            frame(&file, reference(argument(&file, edge.geometry, 1)));
                        let radius = scalar(argument(&file, edge.geometry, 2));
                        center + (x * t.clone().cos() + y * t.sin()) * radius
                    }
                    other => panic!("unadmitted carrier {other}"),
                };
                let encoded = Point3::new(encoded.x * scale, encoded.y * scale, encoded.z * scale);
                assert!(
                    edge.source
                        .curve()
                        .point_at(&parameter.into())
                        .unwrap()
                        .coincides_with(&encoded.into())
                        .unwrap()
                );
            }
        }
        for wire in body.wires() {
            assert_eq!(name(&file, wire.entity), "EDGE_LOOP");
            let outer = wire.source.face().outer() == Some(wire.source);
            assert_eq!(
                name(&file, wire.bound),
                if outer {
                    "FACE_OUTER_BOUND"
                } else {
                    "FACE_BOUND"
                }
            );
            boolean(argument(&file, wire.bound, 2), true);
            assert_eq!(reference(argument(&file, wire.bound, 1)), wire.entity);
            assert_eq!(
                refs(argument(&file, wire.entity, 1)),
                wire.source
                    .edge_uses()
                    .map(|u| body
                        .edge_uses()
                        .iter()
                        .find(|e| e.source == u)
                        .unwrap()
                        .entity)
                    .collect::<Vec<_>>()
            );
        }
        for use_ in body.edge_uses() {
            assert_eq!(name(&file, use_.entity), "ORIENTED_EDGE");
            assert_eq!(
                reference(argument(&file, use_.entity, 3)),
                edge_id(use_.source.edge())
            );
            boolean(
                argument(&file, use_.entity, 4),
                use_.source.direction() == Direction::Forward,
            );
        }
        for face in body.faces() {
            assert_eq!(name(&file, face.entity), "ADVANCED_FACE");
            assert_eq!(reference(argument(&file, face.entity, 2)), face.surface);
            boolean(
                argument(&file, face.entity, 3),
                face.orientation == Orientation::Forward,
            );
            if face.source.surface().kind() != SurfaceKind::Extrusion {
                assert_eq!(face.orientation, face.source.orientation());
            }
            let wires = face
                .source
                .outer()
                .into_iter()
                .chain(face.source.inner())
                .collect::<Vec<_>>();
            assert_eq!(
                refs(argument(&file, face.entity, 1)),
                wires
                    .iter()
                    .map(|w| body.wires().iter().find(|e| e.source == *w).unwrap().bound)
                    .collect::<Vec<_>>()
            );
            let (origin, x, y, z) = frame(&file, reference(argument(&file, face.surface, 1)));
            for use_ in wires.iter().flat_map(|w| w.edge_uses()) {
                let pcurve = use_.pcurve();
                let a = pcurve.domain_start().unwrap();
                let b = pcurve.domain_end().unwrap();
                let t = (a.try_real().unwrap() + b.try_real().unwrap()) * ratio(1, 2);
                let uv = pcurve.point_at(&t.into()).unwrap();
                let uv = uv.coordinates().unwrap();
                let uv = Point2::new(uv.x().clone(), uv.y().clone());
                let native = face.source.surface().point_at(&uv).unwrap();
                let native_uv = uv.clone();
                let uv = face.parameters.map(&uv);
                let encoded = match name(&file, face.surface) {
                    "PLANE" => origin.clone() + x.clone() * &uv.x + y.clone() * &uv.y,
                    "CYLINDRICAL_SURFACE" => {
                        origin.clone()
                            + (x.clone() * &uv.x.clone().cos() + y.clone() * &uv.x.clone().sin())
                                * scalar(argument(&file, face.surface, 2))
                            + z.clone() * &uv.y
                    }
                    "TOROIDAL_SURFACE" => {
                        let major = scalar(argument(&file, face.surface, 2));
                        let minor = scalar(argument(&file, face.surface, 3));
                        origin.clone()
                            + (x.clone() * uv.x.clone().cos() + y.clone() * uv.x.clone().sin())
                                * (major + &minor * uv.y.clone().cos())
                            + z.clone() * minor * uv.y.clone().sin()
                    }
                    other => panic!("unadmitted surface {other}"),
                };
                equal_point(
                    &native,
                    &Point3::new(encoded.x * scale, encoded.y * scale, encoded.z * scale),
                );
                if face.source.surface().kind() == SurfaceKind::Extrusion {
                    // Independently compare oriented geometric normals, rather
                    // than assuming that changing charts preserves face sense.
                    let mut native_normal = face.source.surface().normal_at(&native_uv).unwrap();
                    let mut encoded_normal = match name(&file, face.surface) {
                        "PLANE" => x.cross(&y),
                        "CYLINDRICAL_SURFACE" => {
                            x.clone() * uv.x.clone().cos() + y.clone() * uv.x.sin()
                        }
                        _ => unreachable!(),
                    };
                    if face.source.orientation() == Orientation::Reversed {
                        native_normal = -native_normal;
                    }
                    if face.orientation == Orientation::Reversed {
                        encoded_normal = -encoded_normal;
                    }
                    equal(
                        &native_normal.cross(&encoded_normal).norm_squared(),
                        &Real::zero(),
                    );
                    assert_eq!(
                        compare_reals(
                            &native_normal.dot(&encoded_normal),
                            &Real::zero(),
                            STRICT_PREDICATES
                        )
                        .value(),
                        Some(Ordering::Greater)
                    );
                }
            }
        }
    }
}

pub fn export<'a>(inputs: &[StepBody<'a>], options: &StepWriteOptions<'_>) -> AdvancedStep<'a> {
    let output = write_advanced(inputs, options, &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(output.bodies().len(), inputs.len());
    assert_eq!(
        write_advanced(inputs, options, &context())
            .result
            .unwrap()
            .unwrap()
            .text(),
        output.text()
    );
    graph(&output, &options.length_unit.metres_per_unit);
    output
}
pub fn reimport(output: &AdvancedStep<'_>) -> ImportedRepresentation {
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    let imported = file
        .import_representation(
            output.representation(),
            &ImportLimits::default(),
            &context(),
        )
        .result
        .unwrap()
        .unwrap();
    assert_eq!(imported.bodies().len(), output.bodies().len());
    for (source, body) in output.bodies().iter().zip(imported.bodies()) {
        equal(
            &body.solid().volume().unwrap(),
            &source.source().volume().unwrap(),
        );
        let counts = body.model().counts();
        assert_eq!(
            (
                counts.vertices,
                counts.edges,
                counts.edge_uses,
                counts.wires,
                counts.faces
            ),
            (
                source.vertices().len(),
                source.edges().len(),
                source.edge_uses().len(),
                source.wires().len(),
                source.faces().len()
            )
        );
        let native = body.model().to_json().unwrap();
        let replay = RawModel::from_json(&native, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(native, replay.to_json().unwrap());
    }
    imported
}
pub fn check(solid: SolidRef<'_>) -> ImportedRepresentation {
    reimport(&export(
        &[StepBody {
            solid,
            name: "exact",
        }],
        &StepWriteOptions::new("exact.step", "2026-10-16T00:00:00"),
    ))
}

pub fn imported_circles() {
    for text in [
        cylinder::CYLINDER.to_string(),
        cylinder::partitioned(),
        cylinder::reverse_edge(cylinder::CYLINDER, 21),
        cylinder::reverse_edge(&cylinder::partitioned(), 21),
        cylinder::reverse_edge(cylinder::CYLINDER, 30),
        supplied::supplied(cylinder::CYLINDER),
        supplied::supplied(&cylinder::partitioned()),
        include_str!("../fixtures/negative-circle-axes.step").to_string(),
        include_str!("../fixtures/negative-cylinder-axis.step").to_string(),
        include_str!("../fixtures/negative-cap-axes.step").to_string(),
    ] {
        let original = cylinder::import(&text).unwrap();
        let replay = check(original.bodies()[0].solid());
        cylinder::check(replay.bodies()[0].model());
    }
}

pub fn frames_and_edits() {
    for text in [
        cylinder::CYLINDER,
        include_str!("../fixtures/oblique-cylinder.step"),
        include_str!("../fixtures/radical-frame-cylinder.step"),
    ] {
        let imported = cylinder::import(text).unwrap();
        let model = imported.bodies()[0].model();
        check(model.solids().next().unwrap());
        let edge = model
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
            .unwrap();
        let (a, b) = edge.domain().try_real_endpoints().unwrap();
        let split = edge
            .split(((a + b) * ratio(1, 2)).into())
            .unwrap()
            .into_model();
        let edge = split
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::Line)
            .unwrap();
        let (a, b) = edge.domain().try_real_endpoints().unwrap();
        let split = edge
            .split(((a + b) * ratio(1, 2)).into())
            .unwrap()
            .into_model();
        for transform in [
            Matrix4::affine_translation([20.into(), (-3).into(), 2.into()]),
            Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]),
        ] {
            let moved = split.transformed(&transform).unwrap();
            let replay = check(moved.solids().next().unwrap());
            equal(
                &replay.bodies()[0].solid().volume().unwrap(),
                &imported.bodies()[0].solid().volume().unwrap(),
            );
        }
    }
}

pub fn planar_holes() {
    for text in [
        include_str!("../fixtures/faceted-box.step"),
        include_str!("../fixtures/faceted-two-boxes.step"),
        include_str!("../fixtures/faceted-through-hole.step"),
        include_str!("../fixtures/faceted-tetrahedron.step"),
    ] {
        let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
        let imported = file
            .import_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        let inputs = imported
            .bodies()
            .iter()
            .map(|b| StepBody {
                solid: b.solid(),
                name: b.name(),
            })
            .collect::<Vec<_>>();
        reimport(&export(
            &inputs,
            &StepWriteOptions::new("planar.step", "2026-10-16T00:00:00"),
        ));
    }
    let original =
        builder::cuboid(Point3::origin(), Point3::new(2.into(), 3.into(), 4.into())).unwrap();
    let split = original
        .model()
        .edges()
        .next()
        .unwrap()
        .split(ratio(1, 8).into())
        .unwrap();
    let shear =
        Matrix4::from_row_major([1, 1, 0, 3, 0, 1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from));
    let moved = split.model().transformed(&shear).unwrap();
    let replay = check(moved.solids().next().unwrap());
    equal(
        &replay.bodies()[0].solid().volume().unwrap(),
        &Real::from(48),
    );
}

pub fn units_and_occurrences() {
    use hyperbrep_io::step::WriteLengthUnit;
    let native = builder::cylinder(5.into(), 7.into()).unwrap();
    for scale in [Real::one(), ratio(1, 1000), ratio(127, 5000)] {
        // Direct dimension construction remains a separate exact unit workflow.
        let scaled = builder::cylinder(Real::from(5) * &scale, Real::from(7) * &scale).unwrap();
        let input = StepBody {
            solid: scaled.bodies().next().unwrap(),
            name: "same 'body' \\ Ω🙂\n",
        };
        let mut options = StepWriteOptions::new("analytic'\\Ω🙂\n.step", "2026-10-16T00:00:00");
        options.length_unit = WriteLengthUnit {
            name: "explicit unit",
            metres_per_unit: scale,
        };
        let output = export(&[input, input], &options);
        let imported = reimport(&output);
        for body in imported.bodies() {
            assert_eq!(body.name(), input.name);
        }
        let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
        assert_eq!(file.header().name, options.file_name);
    }
    let gap = ratio(1, 1_000_000) * ratio(1, 1_000_000);
    let nearby = native
        .transformed(&Matrix4::affine_translation([
            Real::from(10) + &gap,
            0.into(),
            0.into(),
        ]))
        .unwrap();
    let output = export(
        &[
            StepBody {
                solid: native.bodies().next().unwrap(),
                name: "a",
            },
            StepBody {
                solid: nearby.bodies().next().unwrap(),
                name: "b",
            },
        ],
        &StepWriteOptions::new("gap.step", "2026-10-16T00:00:00"),
    );
    let imported = reimport(&output);
    let probe = Point3::new(Real::from(5) + gap * ratio(1, 2), 0.into(), 3.into());
    for body in imported.bodies() {
        assert_eq!(
            body.solid().classify_point(&probe).unwrap(),
            SolidPointLocation::Outside
        );
    }
}

pub fn scaled_units() {
    use hyperbrep_io::step::WriteLengthUnit;
    let native = builder::cylinder(5.into(), 7.into()).unwrap();
    for scale in [ratio(1, 1000), ratio(127, 5000)] {
        for reflected in [false, true] {
            let scaled = native
                .transformed(&Matrix4::affine_nonuniform_scale([
                    if reflected {
                        -scale.clone()
                    } else {
                        scale.clone()
                    },
                    scale.clone(),
                    scale.clone(),
                ]))
                .unwrap();
            let mut options = StepWriteOptions::new("scaled.step", "2026-10-16T00:00:00");
            options.length_unit = WriteLengthUnit {
                name: "exact unit",
                metres_per_unit: scale.clone(),
            };
            let output = export(
                &[StepBody {
                    solid: scaled.bodies().next().unwrap(),
                    name: "scaled",
                }],
                &options,
            );
            let imported = reimport(&output);
            equal(
                &imported.bodies()[0].solid().volume().unwrap(),
                &(Real::from(175) * Real::pi() * &scale * &scale * &scale),
            );
            let replay = RawModel::from_json(
                &imported.bodies()[0].model().to_json().unwrap(),
                &ArchiveLimits::default(),
            )
            .unwrap()
            .validate()
            .unwrap();
            let recovered = replay
                .transformed(&Matrix4::affine_nonuniform_scale([
                    (Real::one() / &scale).unwrap(),
                    (Real::one() / &scale).unwrap(),
                    (Real::one() / &scale).unwrap(),
                ]))
                .unwrap();
            let result = BodySet::from_solid(recovered.solids().next().unwrap());
            assert!(boolean::difference(&result, &native).unwrap().is_empty());
        }
    }
}

pub fn composition() {
    let original = builder::cylinder(5.into(), 7.into()).unwrap();
    let imported = check(original.bodies().next().unwrap());
    cylinder::check(imported.bodies()[0].model());
    let a = BodySet::from_solid(imported.bodies()[0].solid());
    let empty = BodySet::empty();
    let a = boolean::union(&empty, &a).unwrap();
    let b = a
        .transformed(&Matrix4::affine_translation([
            20.into(),
            0.into(),
            0.into(),
        ]))
        .unwrap();
    let pair = boolean::union(&a, &b).unwrap();
    let inputs = pair
        .bodies()
        .map(|solid| StepBody {
            solid,
            name: "disconnected",
        })
        .collect::<Vec<_>>();
    let imported = reimport(&export(
        &inputs,
        &StepWriteOptions::new("pair.step", "2026-10-16T00:00:00"),
    ));
    assert_eq!(imported.bodies().len(), 2);
    let rest = boolean::difference(&pair, &a).unwrap();
    let none = boolean::difference(&rest, &b).unwrap();
    assert!(none.is_empty());
    assert!(boolean::intersection(&a, &none).unwrap().is_empty());
    let recovered = boolean::union(&none, &a).unwrap();
    let replay = check(recovered.bodies().next().unwrap());
    cylinder::check(replay.bodies()[0].model());
}

pub fn failures_and_recovery() {
    use hyperbrep_io::step::WriteErrorKind;
    let native = builder::cylinder(5.into(), 7.into()).unwrap();
    let options = StepWriteOptions::new("failure.step", "2026-10-16T00:00:00");
    let good = StepBody {
        solid: native.bodies().next().unwrap(),
        name: "good",
    };
    let sphere = builder::sphere(Real::one()).unwrap();
    let cavity = builder::sphere_with_voids(
        3.into(),
        &[builder::SphereVoid {
            center: Point3::origin(),
            radius: Real::one(),
        }],
    )
    .unwrap();
    let third = builder::cylinder(ratio(1, 3), Real::one()).unwrap();
    let irrational = builder::cylinder(Real::from(2).sqrt().unwrap(), Real::one()).unwrap();
    for (source, kind) in [(&sphere, 0), (&cavity, 1), (&third, 2), (&irrational, 3)] {
        let error = write_advanced(
            &[
                good,
                StepBody {
                    solid: source.bodies().next().unwrap(),
                    name: "bad",
                },
            ],
            &options,
            &context(),
        )
        .result
        .unwrap()
        .unwrap_err();
        assert_eq!(error.body, Some(1));
        assert!(error.entity.is_some());
        assert!(
            matches!(
                (kind, &error.kind),
                (0, WriteErrorKind::UnsupportedSurface(SurfaceKind::Sphere))
                    | (1, WriteErrorKind::VoidShells)
                    | (2, WriteErrorKind::NonTerminatingDecimal)
                    | (3, WriteErrorKind::NonRational)
            ),
            "{error:?}"
        );
        check(good.solid);
    }
    for value in [Real::zero(), -Real::one(), Real::pi(), ratio(1, 3)] {
        let mut changed = options.clone();
        changed.length_unit.metres_per_unit = value;
        let error = write_advanced(&[good], &changed, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert_eq!(error.body, None);
        assert!(matches!(
            error.kind,
            WriteErrorKind::Invalid(_)
                | WriteErrorKind::NonRational
                | WriteErrorKind::NonTerminatingDecimal
        ));
    }
    assert!(matches!(
        write_advanced(&[], &options, &context())
            .result
            .unwrap()
            .unwrap_err()
            .kind,
        WriteErrorKind::Invalid(_)
    ));
    let normal = export(&[good], &options).into_text();
    let approximate =
        hypercurve::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, || {
            write_advanced(&[good], &options, &context())
                .result
                .unwrap()
                .unwrap()
                .into_text()
        });
    assert_eq!(approximate.certified().unwrap(), normal);
}

pub fn limits() {
    use hyperbrep_io::step::{WriteErrorKind, WriteResource};
    let native = builder::cylinder(5.into(), 7.into()).unwrap();
    let input = StepBody {
        solid: native.bodies().next().unwrap(),
        name: "limit",
    };
    let mut options = StepWriteOptions::new("limit.step", "2026-10-16T00:00:00");
    let zero = OperationContext::strict(WorkBudget {
        refinements: 0,
        promotions: 0,
    });
    let output = write_advanced(&[input], &options, &zero)
        .result
        .unwrap()
        .unwrap();
    options.limits.output_bytes = output.text().len();
    options.limits.entities = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())
        .unwrap()
        .syntax()
        .entities()
        .len();
    options.limits.edge_uses = output.bodies()[0].edge_uses().len();
    for resource in [
        WriteResource::OutputBytes,
        WriteResource::Entities,
        WriteResource::EdgeUses,
        WriteResource::NumberBits,
        WriteResource::DecimalDigits,
        WriteResource::DecimalExponent,
    ] {
        let mut small = options.clone();
        match resource {
            WriteResource::OutputBytes => small.limits.output_bytes -= 1,
            WriteResource::Entities => small.limits.entities -= 1,
            WriteResource::EdgeUses => small.limits.edge_uses -= 1,
            WriteResource::NumberBits => small.limits.number_bits = 0,
            WriteResource::DecimalDigits => small.limits.number.digits = 0,
            WriteResource::DecimalExponent => {
                small.length_unit.metres_per_unit = ratio(1, 1000);
                small.limits.number.exponent = 2;
            }
            _ => unreachable!(),
        }
        let error = write_advanced(&[input], &small, &zero)
            .result
            .unwrap()
            .unwrap_err();
        assert!(
            matches!(error.kind,WriteErrorKind::Limit{resource:actual,..} if actual==resource),
            "{error:?}"
        );
        assert_eq!(
            write_advanced(&[input], &options, &zero)
                .result
                .unwrap()
                .unwrap()
                .text(),
            output.text()
        );
    }
}
