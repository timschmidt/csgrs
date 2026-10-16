//! Exact assertions shared by native tests and an actual WASM consumer.
use hyperbrep::*;
use hyperbrep_io::step::{
    Error, ErrorKind, ImportLimits, ImportedRepresentation, Resource, StepBody, StepFile,
    StepReadOptions, StepWriteOptions, WriteErrorKind, write_faceted,
};
use hyperlimit::{PredicateOutcome, compare_reals};
use std::cmp::Ordering;

pub const CYLINDER: &str = include_str!("../fixtures/exact-cylinder.step");
const OCCT: &str = include_str!("../fixtures/occt-cylinder-off-carrier.step");

pub fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    })
}
pub fn read(text: &str) -> StepFile<'_> {
    StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap()
}
pub fn import(text: &str) -> Result<ImportedRepresentation, Error> {
    read(text)
        .import_representation(10, &ImportLimits::default(), &context())
        .result
        .unwrap()
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
pub fn argument(text: &str, id: u64, index: usize, value: &str) -> String {
    let file = read(text);
    let span = file
        .syntax()
        .entity(id)
        .unwrap()
        .components()
        .next()
        .unwrap()
        .arguments()
        .nth(index)
        .unwrap()
        .span();
    let mut result = text.to_string();
    result.replace_range(span.start..span.end, value);
    result
}
pub fn append(text: &str, records: &str) -> String {
    let mut result = text.to_string();
    result.insert_str(result.rfind("ENDSEC;").unwrap(), records);
    result
}
pub fn reverse_edge(text: &str, edge: u64) -> String {
    use hyperbrep_io::part21::ValueKind;
    let file = read(text);
    let mut result = text.to_string();
    for entity in file.syntax().entities() {
        let record = entity.components().next().unwrap();
        let args = record.arguments().collect::<Vec<_>>();
        if record.name() == "ORIENTED_EDGE" && args[3].kind() == ValueKind::Reference(edge) {
            let flag = if args[4].kind() == ValueKind::Enumeration("T") {
                ".F."
            } else {
                ".T."
            };
            result = argument(&result, entity.id(), 4, flag);
        } else if entity.id() == edge {
            let id = |index: usize| match args[index].kind() {
                ValueKind::Reference(id) => format!("#{id}"),
                _ => panic!("vertex reference"),
            };
            result = argument(
                &argument(&argument(&result, edge, 1, &id(2)), edge, 2, &id(1)),
                edge,
                4,
                ".F.",
            );
        }
    }
    result
}

pub fn partitioned() -> String {
    let mut text = argument(CYLINDER, 16, 1, "(#17,#117,#50,#59)");
    text = argument(&text, 21, 2, "#122");
    text = argument(&text, 38, 2, "#131");
    text = argument(&text, 19, 1, "(#20,#29,#37,#144)");
    text = argument(&text, 52, 1, "(#53,#153)");
    text = argument(&text, 61, 1, "(#62,#162)");
    append(
        &text,
        "
#117=ADVANCED_FACE('',(#118),#45,.T.);
#118=FACE_BOUND('',#119,.T.);
#119=EDGE_LOOP('',(#120,#129,#137,#44));
#120=ORIENTED_EDGE('',*,*,#121,.F.);
#121=EDGE_CURVE('',#122,#22,#24,.T.);
#122=VERTEX_POINT('',#123);
#123=CARTESIAN_POINT('',(-3.,-4.,7.));
#129=ORIENTED_EDGE('',*,*,#130,.F.);
#130=EDGE_CURVE('',#131,#122,#133,.T.);
#131=VERTEX_POINT('',#132);
#132=CARTESIAN_POINT('',(-3.,-4.,0.));
#133=LINE('',#134,#35);
#134=CARTESIAN_POINT('',(-3.,-4.,0.));
#137=ORIENTED_EDGE('',*,*,#138,.T.);
#138=EDGE_CURVE('',#131,#31,#39,.T.);
#144=ORIENTED_EDGE('',*,*,#130,.T.);
#153=ORIENTED_EDGE('',*,*,#121,.T.);
#162=ORIENTED_EDGE('',*,*,#138,.F.);
",
    )
}

pub fn check(model: &Model) {
    assert_eq!(model.solids().count(), 1);
    let solid = model.solids().next().unwrap();
    equal(&solid.volume().unwrap(), &(Real::from(175) * Real::pi()));
    let area = model
        .faces()
        .map(|face| face.area().unwrap())
        .fold(Real::zero(), |a, b| a + b);
    equal(&area, &(Real::from(120) * Real::pi()));
    for (point, location) in [
        (
            Point3::new(0.into(), 0.into(), 3.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(3.into(), 4.into(), 3.into()),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(6.into(), 0.into(), 3.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(0.into(), 0.into(), 8.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(0.into(), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), location);
    }
    let gap = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    for (radius, location) in [
        (Real::from(5) - &gap, SolidPointLocation::Inside),
        (Real::from(5) + &gap, SolidPointLocation::Outside),
    ] {
        assert_eq!(
            solid
                .classify_point(&Point3::new(radius, 0.into(), 3.into()))
                .unwrap(),
            location
        );
    }
}
pub fn replay(model: &Model) -> Model {
    let text = model.to_json().unwrap();
    let replay = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.to_json().unwrap(), text);
    replay
}

pub fn closed_circle() {
    for _ in 0..2 {
        let result = import(CYLINDER).unwrap();
        let body = &result.bodies()[0];
        check(body.model());
        check(&replay(body.model()));
        let c = body.model().counts();
        assert_eq!((c.vertices, c.edges, c.edge_uses, c.faces), (2, 3, 6, 3));
        assert!(body.model().vertices().all(|v| v.edges().count() == 2));
        assert_eq!(
            body.model()
                .faces()
                .filter(|f| f.surface().kind() == SurfaceKind::Cylinder)
                .count(),
            1
        );
        assert_eq!(
            body.model()
                .edges()
                .filter(|e| e.curve().kind() == Curve3Kind::CircleArc)
                .count(),
            2
        );
        assert!(body.model().edges().all(|e| e.uses().count() == 2));
    }
}

pub fn parameters() {
    for reverse in [false, true] {
        let text = if reverse {
            reverse_edge(CYLINDER, 21)
        } else {
            CYLINDER.to_string()
        };
        let result = import(&text).unwrap();
        let body = &result.bodies()[0];
        let edge = body.explicit_edges().find(|e| e.source == 21).unwrap();
        assert_eq!((edge.geometry, edge.master), (24, 24));
        assert_eq!(edge.parameters.len(), 1);
        assert_eq!(edge.edge.start(), edge.edge.end());
        let map = &edge.parameters[0];
        equal(
            &(&map.end - &map.start),
            &if reverse { -Real::tau() } else { Real::tau() },
        );
        let domain = edge.edge.curve().domain().clone();
        for (i, (x, y)) in [(3, 4), (-4, 3), (-3, -4), (4, -3), (3, 4)]
            .into_iter()
            .enumerate()
        {
            let fraction = (Real::from(i as u32) / Real::from(4)).unwrap();
            let parameter = domain.start() + (domain.end() - domain.start()) * &fraction;
            let point = edge
                .edge
                .curve()
                .point_at(&parameter.clone().into())
                .unwrap();
            let expected = if reverse {
                [(3, 4), (4, -3), (-3, -4), (-4, 3), (3, 4)][i]
            } else {
                (x, y)
            };
            assert!(
                point
                    .coincides_with(
                        &Point3::new(expected.0.into(), expected.1.into(), 7.into()).into()
                    )
                    .unwrap()
            );
            let source = &map.start + (&map.end - &map.start) * fraction;
            let coordinates = point.try_coordinates().unwrap();
            equal(&coordinates.x, &(Real::from(5) * source.clone().cos()));
            equal(&coordinates.y, &(Real::from(5) * source.sin()));
            for use_ in edge.edge.uses() {
                let uv_parameter = use_.pcurve_parameter_at(&parameter.clone().into()).unwrap();
                assert_eq!(
                    use_.edge_parameter_at(&uv_parameter)
                        .unwrap()
                        .compare(&parameter.clone().into())
                        .unwrap(),
                    Ordering::Equal
                );
            }
        }
        for use_ in edge.edge.uses() {
            let a = use_
                .pcurve_parameter_at(&domain.start().clone().into())
                .unwrap();
            let b = use_
                .pcurve_parameter_at(&domain.end().clone().into())
                .unwrap();
            assert_ne!(a.compare(&b).unwrap(), Ordering::Equal);
        }
        let seam = body.explicit_edges().find(|e| e.source == 30).unwrap();
        equal(&seam.parameters[0].start, &Real::zero());
        equal(
            &seam.parameters[0].end,
            &(Real::from(7) / Real::from(2)).unwrap(),
        );
        let us = seam
            .edge
            .uses()
            .map(|use_| {
                use_.pcurve()
                    .point_at(&use_.pcurve().domain_start().unwrap())
                    .unwrap()
                    .coordinates()
                    .unwrap()
                    .x()
                    .clone()
            })
            .collect::<Vec<_>>();
        assert_eq!(us.len(), 2);
        let difference = &us[0] - &us[1];
        assert!(
            matches!(
                compare_reals(&difference, &Real::tau(), STRICT_PREDICATES),
                PredicateOutcome::Decided {
                    value: Ordering::Equal,
                    ..
                }
            ) || matches!(
                compare_reals(&difference, &-Real::tau(), STRICT_PREDICATES),
                PredicateOutcome::Decided {
                    value: Ordering::Equal,
                    ..
                }
            )
        );
    }
}

pub fn representations() {
    let partitioned = partitioned();
    let mut rotated = CYLINDER.to_string();
    for id in [28, 43] {
        rotated = argument(&rotated, id, 1, "(0.,1.,0.)");
    }
    rotated = argument(&rotated, 49, 1, "(0.,-1.,0.)");
    for id in [58, 67] {
        rotated = argument(&rotated, id, 1, "(-1.,0.,0.)");
    }
    let reversed_bound = argument(
        &argument(CYLINDER, 19, 1, "(#44,#37,#29,#20)"),
        18,
        2,
        ".F.",
    );
    let reversed_bound = [20, 29, 37, 44]
        .into_iter()
        .fold(reversed_bound, |text, id| {
            let value = if [20, 29].contains(&id) { ".T." } else { ".F." };
            argument(&text, id, 4, value)
        });
    for text in [
        include_str!("../fixtures/negative-circle-axes.step").to_string(),
        include_str!("../fixtures/negative-cylinder-axis.step").to_string(),
        include_str!("../fixtures/negative-cap-axes.step").to_string(),
        partitioned.clone(),
        reverse_edge(&partitioned, 21),
        reverse_edge(CYLINDER, 21),
        reverse_edge(CYLINDER, 30),
        rotated,
        reversed_bound,
        argument(CYLINDER, 19, 1, "(#29,#37,#44,#20)"),
        argument(CYLINDER, 16, 1, "(#59,#50,#17)"),
    ] {
        let result = import(&text).unwrap();
        check(&replay(result.bodies()[0].model()));
    }
    for (x, y) in [
        (3, 4),
        (-3, 4),
        (-3, -4),
        (3, -4),
        (5, 0),
        (0, 5),
        (-5, 0),
        (0, -5),
    ] {
        let text = argument(
            &argument(
                &argument(CYLINDER, 23, 1, &format!("({x}.,{y}.,7.)")),
                32,
                1,
                &format!("({x}.,{y}.,0.)"),
            ),
            34,
            1,
            &format!("({x}.,{y}.,0.)"),
        );
        check(import(&text).unwrap().bodies()[0].model());
    }
}

pub fn oblique_frames() {
    let fraction = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
    for (text, height, inside, boundary, outside, beyond) in [
        (
            include_str!("../fixtures/oblique-cylinder.step"),
            Real::from(7),
            Point3::new(0.into(), fraction(-12, 5), fraction(9, 5)),
            Point3::new(3.into(), 0.into(), 5.into()),
            Point3::new(6.into(), 0.into(), 5.into()),
            Point3::new(0.into(), fraction(-32, 5), fraction(24, 5)),
        ),
        (
            include_str!("../fixtures/radical-frame-cylinder.step"),
            Real::from(7) * Real::from(2).sqrt().unwrap(),
            Point3::new(0.into(), 3.into(), 3.into()),
            Point3::new(5.into(), 3.into(), 3.into()),
            Point3::new(6.into(), 3.into(), 3.into()),
            Point3::new(0.into(), 8.into(), 8.into()),
        ),
    ] {
        let imported = import(text).unwrap();
        let source = imported.bodies()[0].model();
        let decoded = replay(source);
        for model in [source, &decoded] {
            let counts = model.counts();
            assert_eq!(
                (
                    counts.vertices,
                    counts.edges,
                    counts.edge_uses,
                    counts.faces
                ),
                (2, 3, 6, 3)
            );
            assert_eq!(model.solids().count(), 1);
            assert!(model.edges().all(|edge| edge.uses().count() == 2));
            assert_eq!(
                model
                    .edges()
                    .filter(|e| e.curve().kind() == Curve3Kind::CircleArc)
                    .count(),
                2
            );
            assert_eq!(
                model
                    .faces()
                    .filter(|f| f.surface().kind() == SurfaceKind::Cylinder)
                    .count(),
                1
            );
            let solid = model.solids().next().unwrap();
            equal(
                &solid.volume().unwrap(),
                &(Real::from(25) * Real::pi() * &height),
            );
            equal(
                &model
                    .faces()
                    .map(|face| face.area().unwrap())
                    .fold(Real::zero(), |a, b| a + b),
                &((Real::from(50) + Real::from(10) * &height) * Real::pi()),
            );
            for (point, expected) in [
                (&inside, SolidPointLocation::Inside),
                (&boundary, SolidPointLocation::Boundary),
                (&outside, SolidPointLocation::Outside),
                (&beyond, SolidPointLocation::Outside),
                (
                    &Point3::new(0.into(), 0.into(), 0.into()),
                    SolidPointLocation::Boundary,
                ),
            ] {
                assert_eq!(solid.classify_point(point).unwrap(), expected);
            }
        }
    }
}

pub fn units() {
    for (length, multiplier) in [("$", Real::one()), (".MILLI.", Real::from(1_000_000_000))] {
        let text = CYLINDER.replace("SI_UNIT($,.METRE.)", &format!("SI_UNIT({length},.METRE.)"));
        for (angle, factor) in [("$", Real::one()), (".MILLI.", Real::from(1000))] {
            let text = text.replace("SI_UNIT($,.RADIAN.)", &format!("SI_UNIT({angle},.RADIAN.)"));
            let result = import(&text).unwrap();
            equal(
                &result.bodies()[0].solid().volume().unwrap(),
                &(Real::from(175) * Real::pi() / &multiplier).unwrap(),
            );
            for edge in result.bodies()[0]
                .explicit_edges()
                .filter(|e| e.source != 30)
            {
                equal(
                    &(&edge.parameters[0].end - &edge.parameters[0].start),
                    &(Real::tau() * &factor),
                );
            }
        }
    }
    let text = append(
        &CYLINDER.replace(
            "#70=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
            "#70=(CONVERSION_BASED_UNIT('degree',#80) NAMED_UNIT(#81) PLANE_ANGLE_UNIT());",
        ),
        "#80=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.02),#82);\n#81=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\n#82=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n",
    );
    let result = import(&text).unwrap();
    check(result.bodies()[0].model());
    equal(
        &result.units().plane_angle.as_ref().unwrap().scale_to_si,
        &(Real::one() / Real::from(50)).unwrap(),
    );
    for edge in result.bodies()[0]
        .explicit_edges()
        .filter(|e| e.source != 30)
    {
        equal(
            &(&edge.parameters[0].end - &edge.parameters[0].start),
            &(Real::tau() * Real::from(50)),
        );
    }
}

pub fn composition() {
    let imported = import(CYLINDER).unwrap();
    let model = imported.bodies()[0].model();
    let original = model.to_json().unwrap();
    let circle = model
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
        .unwrap();
    let t = circle.curve().domain().start() + Real::pi();
    let split = circle.split(t.into()).unwrap().into_model();
    check(&replay(&split));
    assert_eq!(
        (
            split.counts().vertices,
            split.counts().edges,
            split.counts().edge_uses
        ),
        (3, 4, 8)
    );
    let split = split
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::Line)
        .unwrap()
        .split((Real::one() / Real::from(2)).unwrap().into())
        .unwrap()
        .into_model();
    check(&replay(&split));
    let reflected = split
        .transformed(&Matrix4::affine_nonuniform_scale([
            -Real::one(),
            Real::one(),
            Real::one(),
        ]))
        .unwrap();
    check(&replay(&reflected));
    let a = BodySet::from_solid(model.solids().next().unwrap());
    let b = a
        .transformed(&Matrix4::affine_translation([
            20.into(),
            0.into(),
            0.into(),
        ]))
        .unwrap();
    let a = boolean::union(&BodySet::empty(), &a).unwrap();
    let pair = boolean::union(&a, &b).unwrap();
    assert_eq!(pair.len(), 2);
    equal(
        &pair
            .bodies()
            .map(|s| s.volume().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        &(Real::from(350) * Real::pi()),
    );
    for (x, inside) in [(0, true), (10, false), (20, true)] {
        assert_eq!(
            pair.bodies().any(|s| s
                .classify_point(&Point3::new(x.into(), 0.into(), 3.into()))
                .unwrap()
                == SolidPointLocation::Inside),
            inside
        );
    }
    let remainder = boolean::difference(&pair, &a).unwrap();
    let empty = boolean::difference(&remainder, &b).unwrap();
    assert!(empty.is_empty());
    assert!(boolean::intersection(&a, &empty).unwrap().is_empty());
    check(&replay(boolean::union(&empty, &a).unwrap().model()));
    assert_eq!(model.to_json().unwrap(), original);
    let artifact = write_faceted(
        &[StepBody {
            solid: model.solids().next().unwrap(),
            name: "curved",
        }],
        &StepWriteOptions::new("curved.step", "2026-10-15T00:00:00"),
        &context(),
    )
    .result
    .unwrap()
    .unwrap_err();
    assert!(matches!(
        artifact.kind,
        WriteErrorKind::UnsupportedSurface(_) | WriteErrorKind::UnsupportedCurve(_)
    ));
}

pub fn failures() {
    for text in [
        OCCT.to_string(),
        argument(CYLINDER, 23, 1, "(3.,4.000000000001,7.)"),
        argument(CYLINDER, 24, 2, "5.000000000001"),
        argument(CYLINDER, 45, 2, "5.000000000001"),
        argument(CYLINDER, 23, 1, "(3.,4.,7.000000000001)"),
        argument(CYLINDER, 49, 1, "(0.,0.,0.)"),
        argument(CYLINDER, 24, 2, "0."),
        argument(CYLINDER, 19, 1, "(#20,#37,#29,#44)"),
        argument(CYLINDER, 53, 4, ".F."),
        argument(CYLINDER, 17, 3, ".F."),
    ] {
        assert!(import(&text).is_err(), "invalid source admitted");
    }
    let external = import(OCCT).unwrap_err();
    assert_eq!(external.entity, Some(24));
    assert!(matches!(
        external.kind,
        ErrorKind::Invalid("point is not on the exact source circle")
    ));
    let gap = argument(CYLINDER, 23, 1, "(3.,4.000000000001,7.)");
    let ambient = hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, || {
        import(&gap)
    });
    assert!(matches!(
        ambient.certified().unwrap().unwrap_err().kind,
        ErrorKind::Invalid(_)
    ));
    let missing = CYLINDER.replace(
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#69,#70))",
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#69))",
    );
    assert!(matches!(
        import(&missing).unwrap_err().kind,
        ErrorKind::Expected("plane-angle unit for angular mapping")
    ));
    let ellipse = CYLINDER.replace("#24=CIRCLE('',#25,5.);", "#24=ELLIPSE('',#25,5.,3.);");
    assert!(matches!(
        import(&ellipse).unwrap_err().kind,
        ErrorKind::Unsupported(_)
    ));
    // Supplied curved representations now reach their referenced definitions.
    // Keep both original malformed inputs and require the specific missing ID.
    for (text, missing) in [
        (
            append(
                &argument(CYLINDER, 21, 3, "#90"),
                "#90=SURFACE_CURVE('',#24,(#91),.CURVE_3D.);\n#91=PCURVE('',#45,#92);\n",
            ),
            92,
        ),
        (
            append(
                &argument(CYLINDER, 21, 3, "#90"),
                "#90=SEAM_CURVE('',#24,(#91,#92),.CURVE_3D.);\n",
            ),
            91,
        ),
    ] {
        assert!(matches!(
            import(&text).unwrap_err().kind,
            ErrorKind::MissingEntity(id) if id == missing
        ));
    }
    let duplicate = append(
        &argument(CYLINDER, 21, 2, "#90"),
        "#90=VERTEX_POINT('distinct',#23);\n",
    );
    assert!(matches!(
        import(&duplicate).unwrap_err().kind,
        ErrorKind::Invalid("distinct circle vertices have the same exact position")
    ));
    check(import(CYLINDER).unwrap().bodies()[0].model());
}

pub fn admission() {
    let two = append(
        &argument(CYLINDER, 10, 1, "(#15,#90)"),
        "#90=MANIFOLD_SOLID_BREP('second',#16);\n#999=UNSELECTED(#9999);\n",
    );
    let result = import(&two).unwrap();
    assert_eq!(result.bodies().len(), 2);
    assert_ne!(
        result.bodies()[0].vertices().next().unwrap().vertex,
        result.bodies()[1].vertices().next().unwrap().vertex
    );
    assert!(result.retained_entities().contains(&999));
    let ids = result
        .interpreted_entities()
        .iter()
        .chain(result.retained_entities())
        .copied()
        .collect::<std::collections::HashSet<_>>();
    assert_eq!(ids.len(), read(&two).syntax().entities().len());
    assert_eq!(
        ids.len(),
        result.interpreted_entities().len() + result.retained_entities().len()
    );
    for limits in [
        ImportLimits {
            edge_uses: 5,
            ..ImportLimits::default()
        },
        ImportLimits {
            scalars: 0,
            ..ImportLimits::default()
        },
        ImportLimits {
            entity_visits: 0,
            ..ImportLimits::default()
        },
        ImportLimits {
            text_bytes: 0,
            ..ImportLimits::default()
        },
    ] {
        assert!(matches!(
            read(CYLINDER)
                .import_representation(10, &limits, &context())
                .result
                .unwrap()
                .unwrap_err()
                .kind,
            ErrorKind::Limit { .. }
        ));
    }
    let file = read(CYLINDER);
    let result = file
        .import_representation(
            10,
            &ImportLimits {
                edge_uses: 6,
                ..ImportLimits::default()
            },
            &context(),
        )
        .result
        .unwrap()
        .unwrap();
    check(result.bodies()[0].model());
    let bad = argument(&two, 90, 1, "#9999");
    assert!(matches!(
        import(&bad).unwrap_err().kind,
        ErrorKind::MissingEntity(9999)
    ));
    let failed = file
        .import_representation(
            10,
            &ImportLimits {
                edge_uses: 5,
                ..ImportLimits::default()
            },
            &context(),
        )
        .result
        .unwrap()
        .unwrap_err();
    assert!(matches!(
        failed.kind,
        ErrorKind::Limit {
            resource: Resource::EdgeUses,
            ..
        }
    ));
    check(import(CYLINDER).unwrap().bodies()[0].model());
}
