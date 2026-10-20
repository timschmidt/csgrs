use hyperbrep::*;
use hyperbrep_io::step::*;

const INDEPENDENT: &str = include_str!("../fixtures/quintic-extrusion.step");

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    })
}
fn equal(a: &Real, b: &Real) {
    assert!((a - b).exact_rational_normal_form().unwrap().is_zero());
}
fn replay(model: &Model) -> Model {
    let text = model.to_json().unwrap();
    let cold = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), text);
    cold
}
fn save(name: &str, text: &str) {
    #[cfg(not(target_arch = "wasm32"))]
    if let Ok(directory) = std::env::var("HYPERBREP_MEASUREMENT_EXPORT_DIR") {
        let directory = std::path::Path::new(&directory);
        std::fs::create_dir_all(directory).unwrap();
        std::fs::write(directory.join(name), text).unwrap();
    }
    #[cfg(target_arch = "wasm32")]
    let _ = (name, text);
}
fn verify(model: &Model, scale: &Real) {
    assert_eq!(model.solids().count(), 1);
    let solid = model.solids().next().unwrap();
    assert_eq!(
        solid.volume(),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    );
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    // Exact Bernstein evaluation, independent of the STEP reader: t=1/2.
    let gap: Real = "1e-12".parse().unwrap();
    for (x, y, expected) in [
        (q(22, 107), q(-412, 107), SolidPointLocation::Boundary),
        (q(22, 107), q(-412, 107) + &gap, SolidPointLocation::Inside),
        (q(22, 107), q(-412, 107) - &gap, SolidPointLocation::Outside),
        (0.into(), 0.into(), SolidPointLocation::Inside),
        (0.into(), 4.into(), SolidPointLocation::Boundary),
        (0.into(), Real::from(4) + &gap, SolidPointLocation::Outside),
        (3.into(), 1.into(), SolidPointLocation::Outside),
    ] {
        assert_eq!(
            solid
                .classify_point(&Point3::new(x * scale, y * scale, q(3, 2) * scale))
                .unwrap(),
            expected
        );
    }
}
fn variant(reverse: bool, milli: bool) -> String {
    let mut text = INDEPENDENT.to_owned();
    if milli {
        text = text.replace("SI_UNIT($,.METRE.)", "SI_UNIT(.MILLI.,.METRE.)");
    }
    if reverse {
        text = text
            .replace(
                "(#10,#110,#111,#112,#113,#12)",
                "(#12,#113,#112,#111,#110,#10)",
            )
            .replace(
                "(#13,#114,#115,#116,#117,#15)",
                "(#15,#117,#116,#115,#114,#13)",
            )
            .replace("(1.,2.,3.,4.,5.,1.)", "(1.,5.,4.,3.,2.,1.)")
            .replace(
                "EDGE_CURVE('',#20,#21,#30,.T.)",
                "EDGE_CURVE('',#20,#21,#30,.F.)",
            )
            .replace(
                "EDGE_CURVE('',#22,#23,#31,.T.)",
                "EDGE_CURVE('',#22,#23,#31,.F.)",
            )
            .replace(
                "ADVANCED_FACE('',(#85),#60,.T.)",
                "ADVANCED_FACE('',(#85),#60,.F.)",
            );
    }
    text
}
fn import(text: &str) -> ImportedRepresentation {
    StepFile::read(text.as_bytes(), &StepReadOptions::default())
        .unwrap()
        .import_representation(96, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap()
}

pub fn independent_quintic_step_admits_and_replays_without_measurement() {
    for reverse in [false, true] {
        for milli in [false, true] {
            let text = variant(reverse, milli);
            let scale = if milli { q(1, 1000) } else { Real::one() };
            save(&format!("quintic-{reverse}-{milli}-input.step"), &text);
            let imported = import(&text);
            let body = &imported.bodies()[0];
            verify(body.model(), &scale);
            verify(&replay(body.model()), &scale);
            save(
                &format!("quintic-{reverse}-{milli}-native.json"),
                &body.model().to_json().unwrap(),
            );
            for source in body
                .explicit_edges()
                .filter(|edge| edge.source == 40 || edge.source == 42)
            {
                let geometry = source.edge.curve();
                let spline = geometry.nurbs_geometry().unwrap();
                assert_eq!(spline.degree, 5);
                assert_eq!(spline.control_points.len(), 6);
                assert_eq!(
                    spline.knots,
                    [vec![Real::from(2); 6], vec![Real::from(4); 6]].concat()
                );
                // Native edges follow their topological start/end. Reversed
                // STEP controls and weights are reversed together; the source
                // parameter map below retains the decreasing original chart.
                assert_eq!(
                    spline.weights,
                    &[
                        Real::from(1),
                        2.into(),
                        3.into(),
                        4.into(),
                        5.into(),
                        1.into()
                    ]
                );
                for (control, x) in spline.control_points.iter().zip([
                    q(-2, 1),
                    q(-6, 5),
                    q(-2, 5),
                    q(2, 5),
                    q(6, 5),
                    q(2, 1),
                ]) {
                    equal(&control.x, &(x * &scale));
                }
                equal(
                    &source.parameters[0].start,
                    &Real::from(if reverse { 4 } else { 2 }),
                );
                equal(
                    &source.parameters[0].end,
                    &Real::from(if reverse { 2 } else { 4 }),
                );
            }
            let options = StepWriteOptions::new("quintic.step", "2026-10-18T00:00:00");
            let bodies = [StepBody {
                solid: body.solid(),
                name: "quintic",
            }];
            let output = write_advanced(&bodies, &options, &context())
                .result
                .unwrap()
                .unwrap();
            assert_eq!(
                output.text(),
                write_advanced(&bodies, &options, &context())
                    .result
                    .unwrap()
                    .unwrap()
                    .text()
            );
            save(
                &format!("quintic-{reverse}-{milli}-output.step"),
                output.text(),
            );
            let file =
                StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
            let again = file
                .import_representation(
                    output.representation(),
                    &ImportLimits::default(),
                    &context(),
                )
                .result
                .unwrap()
                .unwrap();
            verify(&replay(again.bodies()[0].model()), &scale);
            let original = BodySet::from_solid(body.solid());
            let cold = replay(again.bodies()[0].model());
            let selected = BodySet::from_solid(cold.solids().next().unwrap());
            assert!(
                boolean::difference(&selected, &original)
                    .unwrap()
                    .is_empty()
            );
            let cutter = selected
                .transformed(&Matrix4::from_row_major([
                    1.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    q(1, 3),
                    scale.clone(),
                    0.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                ]))
                .unwrap();
            let middle = boolean::intersection(&selected, &cutter).unwrap();
            let ends = boolean::difference(&selected, &cutter).unwrap();
            assert_eq!(ends.len(), 2);
            let restored = boolean::union(&middle, &ends).unwrap();
            verify(&replay(restored.model()), &scale);
            assert!(
                boolean::difference(&restored, &selected)
                    .unwrap()
                    .is_empty()
            );
            assert!(
                boolean::difference(&selected, &restored)
                    .unwrap()
                    .is_empty()
            );
        }
    }
}

pub fn malformed_quintic_step_and_budget_failure_cannot_publish_geometry() {
    for text in [
        INDEPENDENT.replace(
            "ADVANCED_FACE('',(#73),#55,.F.)",
            "ADVANCED_FACE('',(#73),#55,.T.)",
        ),
        INDEPENDENT.replace("(-0.4,-4.,0.)", "(-0.4,-4.,1.)"),
        INDEPENDENT.replace("(1.,2.,3.,4.,5.,1.)", "(1.,2.,0.,4.,5.,1.)"),
    ] {
        assert_ne!(text, INDEPENDENT);
        let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
        let result = file.import_representation(96, &ImportLimits::default(), &context());
        assert!(result.result.unwrap().is_err());
    }
    let file = StepFile::read(INDEPENDENT.as_bytes(), &StepReadOptions::default()).unwrap();
    let limits = ImportLimits {
        spline_knots: 11,
        ..ImportLimits::default()
    };
    let error = file
        .import_representation(96, &limits, &context())
        .result
        .unwrap()
        .unwrap_err();
    assert!(matches!(
        error.kind,
        ErrorKind::Limit {
            resource: Resource::SplineKnots,
            limit: 11,
            observed: 12
        }
    ));
    assert_eq!(error.entity, Some(30));
    // Reuse after a rejected import must re-run the complete native admission.
    verify(import(INDEPENDENT).bodies()[0].model(), &Real::one());
}
