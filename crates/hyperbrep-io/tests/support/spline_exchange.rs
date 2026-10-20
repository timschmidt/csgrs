use hyperbrep::*;
use hypercurve::{
    Curve2, CurveFamily2, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2 as CurvePoint2,
    QuadraticBezier2,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: i32, y: i32) -> CurvePoint2 {
    CurvePoint2::from_values(x, y)
}

fn equal(actual: Real, expected: Real) {
    assert!(
        (actual - expected)
            .exact_rational_normal_form()
            .unwrap()
            .is_zero()
    );
}

fn parabola(offset: i32, reverse: bool) -> CurvePath2 {
    // x = -2 + 4t + offset, y = 4 - 16t + 16t²: y = (x-offset)².
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(offset - 2, 4), p(offset, -4), p(offset + 2, 4)).into(),
        LineSeg2::try_new(p(offset + 2, 4), p(offset - 2, 4))
            .unwrap()
            .into(),
    ])
    .unwrap();
    if reverse {
        path.reversed().unwrap()
    } else {
        path
    }
}

fn hole(reverse: bool) -> CurvePath2 {
    let points = [
        CurvePoint2::new(q(-1, 2), 1.into()),
        CurvePoint2::new(q(1, 2), 1.into()),
        CurvePoint2::new(q(1, 2), 2.into()),
        CurvePoint2::new(q(-1, 2), 2.into()),
    ];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    if reverse {
        path.reversed().unwrap()
    } else {
        path
    }
}

fn profile(reverse: bool, components: bool, affine: bool) -> CurveRegion2 {
    let mut paths = vec![parabola(0, reverse)];
    if components {
        paths.extend([hole(reverse), parabola(8, reverse)]);
    }
    let region = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    assert!(region.boundary_loops().iter().any(|boundary| {
        boundary
            .curves()
            .iter()
            .any(|curve| curve.family() == CurveFamily2::Line && curve.geometry().is_none())
    }));
    if affine {
        region
            .transform_affine(
                &(-2).into(),
                &0.into(),
                &0.into(),
                &3.into(),
                &7.into(),
                &(-1).into(),
            )
            .unwrap()
    } else {
        region
    }
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

fn verify(model: &Model, components: bool, affine: bool) {
    let scale = if affine { 6 } else { 1 };
    equal(
        model.solids().map(|solid| solid.volume().unwrap()).sum(),
        ((if components { 61 } else { 32 }) * scale).into(),
    );
    assert_eq!(model.solids().count(), if components { 2 } else { 1 });
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    let gap: Real = "1e-12".parse().unwrap();
    let mut cases = vec![
        (1.into(), 2.into(), SolidPointLocation::Inside),
        (1.into(), 1.into(), SolidPointLocation::Boundary),
        (1.into(), Real::one() + &gap, SolidPointLocation::Inside),
        (1.into(), Real::one() - &gap, SolidPointLocation::Outside),
        (0.into(), 4.into(), SolidPointLocation::Boundary),
        (0.into(), Real::from(4) - &gap, SolidPointLocation::Inside),
        (0.into(), Real::from(4) + &gap, SolidPointLocation::Outside),
        (4.into(), 2.into(), SolidPointLocation::Outside),
    ];
    if components {
        cases.extend([
            (0.into(), q(3, 2), SolidPointLocation::Outside),
            (q(1, 2), q(3, 2), SolidPointLocation::Boundary),
            (q(1, 2) + &gap, q(3, 2), SolidPointLocation::Inside),
            (q(1, 2) - &gap, q(3, 2), SolidPointLocation::Outside),
            (8.into(), 2.into(), SolidPointLocation::Inside),
        ]);
    }
    for (x, y, expected) in cases {
        let (x, y) = if affine {
            (
                Real::from(-2) * x + Real::from(7),
                Real::from(3) * y - Real::one(),
            )
        } else {
            (x, y)
        };
        let point = Point3::new(x, y, q(3, 2));
        let mut location = SolidPointLocation::Outside;
        for solid in model.solids() {
            match solid.classify_point(&point).unwrap() {
                SolidPointLocation::Inside => {
                    location = SolidPointLocation::Inside;
                    break;
                }
                SolidPointLocation::Boundary => location = SolidPointLocation::Boundary,
                SolidPointLocation::Outside => {}
            }
        }
        assert_eq!(location, expected, "{point:?}");
    }
}

use hyperbrep_io::step::*;

pub fn retained_spline_profiles_complete_step_replay_and_repeated_cuts() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    });
    for reverse in [false, true] {
        for components in [false, true] {
            for affine in [false, true] {
                let region = profile(reverse, components, affine);
                let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
                let bodies: Vec<_> = original
                    .bodies()
                    .map(|solid| StepBody {
                        solid,
                        name: "retained spline",
                    })
                    .collect();
                let options = StepWriteOptions::new("spline.step", "2026-10-18T00:00:00");
                let output = write_advanced(&bodies, &options, &context)
                    .result
                    .unwrap()
                    .unwrap();
                assert_eq!(
                    output.text(),
                    write_advanced(&bodies, &options, &context)
                        .result
                        .unwrap()
                        .unwrap()
                        .text()
                );
                let file =
                    StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
                let imported = file
                    .import_representation(
                        output.representation(),
                        &ImportLimits::default(),
                        &context,
                    )
                    .result
                    .unwrap()
                    .unwrap();
                let mut joined = BodySet::empty();
                for imported in imported.bodies() {
                    let cold = replay(imported.model());
                    joined = boolean::union(
                        &joined,
                        &BodySet::from_solid(cold.solids().next().unwrap()),
                    )
                    .unwrap();
                }
                verify(joined.model(), components, affine);
                verify(&replay(joined.model()), components, affine);
                let cutter = builder::extrude_curve_region(&region, 1.into(), 2.into()).unwrap();
                let middle = boolean::intersection(&joined, &cutter).unwrap();
                let removed = boolean::difference(&joined, &cutter).unwrap_or_else(|e| {
                    panic!("reverse={reverse} components={components} affine={affine}: {e:?}")
                });
                let volume =
                    Real::from((if components { 61 } else { 32 }) * if affine { 6 } else { 1 });
                equal(
                    middle.bodies().map(|s| s.volume().unwrap()).sum(),
                    (&volume / Real::from(3)).unwrap(),
                );
                equal(
                    removed.bodies().map(|s| s.volume().unwrap()).sum(),
                    (&volume * Real::from(2) / Real::from(3)).unwrap(),
                );
                let restored = boolean::union(&removed, &middle).unwrap();
                verify(restored.model(), components, affine);
                verify(&replay(restored.model()), components, affine);
                assert!(boolean::difference(&restored, &joined).unwrap().is_empty());
                assert!(boolean::difference(&joined, &restored).unwrap().is_empty());
            }
        }
    }
}

const INDEPENDENT: &str = include_str!("../fixtures/spline-extrusion.step");

fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    })
}

fn import(text: &str, limits: &ImportLimits) -> Result<ImportedRepresentation, Error> {
    StepFile::read(text.as_bytes(), &StepReadOptions::default())
        .unwrap()
        .import_representation(96, limits, &context())
        .result
        .unwrap()
}

fn replace_record(text: &str, id: u64, body: &str) -> String {
    let begin = text.find(&format!("#{id}=")).unwrap();
    let end = begin + text[begin..].find(';').unwrap() + 1;
    format!("{}#{id}={body};{}", &text[..begin], &text[end..])
}

fn rational_record(id: u64, weights: &str) -> String {
    let controls = if id == 30 {
        "#10,#11,#12"
    } else {
        "#13,#14,#15"
    };
    format!(
        "(BOUNDED_CURVE()B_SPLINE_CURVE(2,({controls}),.UNSPECIFIED.,.F.,.U.)B_SPLINE_CURVE_WITH_KNOTS((3,3),(2.,4.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(({weights}))REPRESENTATION_ITEM(''))"
    )
}

fn independent_variant(kind: usize) -> String {
    let mut text = INDEPENDENT.to_owned();
    match kind {
        0 => {}
        1 => {
            // Weights in geometric progression reparameterize the same
            // polynomial parabola by t -> 2t/(1+t), without changing its image.
            for id in [30, 31] {
                text = replace_record(&text, id, &rational_record(id, "1.,2.,4."));
            }
        }
        2 => {
            // Independently specified knot insertion at the chart midpoint.
            text = text.replace("ENDSEC;\nEND-ISO", "#110=CARTESIAN_POINT('',(-1.,0.,0.));\n#111=CARTESIAN_POINT('',(1.,0.,0.));\n#112=CARTESIAN_POINT('',(-1.,0.,3.));\n#113=CARTESIAN_POINT('',(1.,0.,3.));\nENDSEC;\nEND-ISO");
            text = replace_record(
                &text,
                30,
                "B_SPLINE_CURVE_WITH_KNOTS('',2,(#10,#110,#111,#12),.UNSPECIFIED.,.F.,.U.,(3,1,3),(2.,3.,4.),.UNSPECIFIED.)",
            );
            text = replace_record(
                &text,
                31,
                "B_SPLINE_CURVE_WITH_KNOTS('',2,(#13,#112,#113,#15),.UNSPECIFIED.,.F.,.U.,(3,1,3),(2.,3.,4.),.UNSPECIFIED.)",
            );
        }
        3 => {
            text = text
                .replace("(#10,#11,#12)", "(#12,#11,#10)")
                .replace("(#13,#14,#15)", "(#15,#14,#13)");
            text = replace_record(&text, 40, "EDGE_CURVE('',#20,#21,#30,.F.)");
            text = replace_record(&text, 42, "EDGE_CURVE('',#22,#23,#31,.F.)");
            text = replace_record(&text, 86, "ADVANCED_FACE('',(#85),#60,.F.)");
        }
        4 => {
            // Topological trims select the middle half of a larger carrier.
            text = text.replace("ENDSEC;\nEND-ISO", "#110=CARTESIAN_POINT('',(-4.,16.,0.));\n#111=CARTESIAN_POINT('',(0.,-16.,0.));\n#112=CARTESIAN_POINT('',(4.,16.,0.));\n#113=CARTESIAN_POINT('',(-4.,16.,3.));\n#114=CARTESIAN_POINT('',(0.,-16.,3.));\n#115=CARTESIAN_POINT('',(4.,16.,3.));\nENDSEC;\nEND-ISO");
            text = replace_record(
                &text,
                30,
                "B_SPLINE_CURVE_WITH_KNOTS('',2,(#110,#111,#112),.UNSPECIFIED.,.F.,.U.,(3,3),(0.,1.),.UNSPECIFIED.)",
            );
            text = replace_record(
                &text,
                31,
                "B_SPLINE_CURVE_WITH_KNOTS('',2,(#113,#114,#115),.UNSPECIFIED.,.F.,.U.,(3,3),(0.,1.),.UNSPECIFIED.)",
            );
        }
        _ => unreachable!(),
    }
    text
}

pub fn independent_splines_retain_charts_weights_trims_units_and_modeling() {
    for kind in 0..5 {
        for milli in [false, true] {
            let mut text = independent_variant(kind);
            let scale = if milli {
                text = text.replace("SI_UNIT($,.METRE.)", "SI_UNIT(.MILLI.,.METRE.)");
                q(1, 1000)
            } else {
                Real::one()
            };
            let imported = import(&text, &ImportLimits::default())
                .unwrap_or_else(|e| panic!("kind={kind} milli={milli}: {e:?}"));
            let body = &imported.bodies()[0];
            equal(
                body.solid().volume().unwrap(),
                Real::from(32) * &scale * &scale * &scale,
            );
            let cold = replay(body.model());
            let solid = cold.solids().next().unwrap();
            assert_eq!(
                solid
                    .classify_point(&Point3::new(
                        Real::one() * &scale,
                        Real::from(2) * &scale,
                        q(3, 2) * &scale
                    ))
                    .unwrap(),
                SolidPointLocation::Inside
            );
            for source in body.explicit_edges() {
                if source.source == 40 || source.source == 42 {
                    let parameters = &source.parameters[0];
                    equal(
                        parameters.start.clone(),
                        if kind == 3 {
                            4.into()
                        } else if kind == 4 {
                            q(1, 4)
                        } else {
                            2.into()
                        },
                    );
                    equal(
                        parameters.end.clone(),
                        if kind == 3 {
                            2.into()
                        } else if kind == 4 {
                            q(3, 4)
                        } else {
                            4.into()
                        },
                    );
                    assert_eq!(parameters.curve, if source.source == 40 { 30 } else { 31 });
                    let geometry = source.edge.curve();
                    let view = geometry.nurbs_geometry().unwrap();
                    assert_eq!(view.degree, 2);
                    if kind == 1 {
                        assert_eq!(view.weights, &[Real::from(1), 2.into(), 4.into()]);
                    }
                    if kind == 2 {
                        assert_eq!(
                            view.knots,
                            &[
                                Real::from(2),
                                2.into(),
                                2.into(),
                                3.into(),
                                4.into(),
                                4.into(),
                                4.into()
                            ]
                        );
                    }
                }
            }
            let face = body.faces().find(|face| face.source == 86).unwrap();
            let uv = Point2::new(if kind == 4 { q(1, 2) } else { 3.into() }, q(1, 2));
            let source = face.parameters.map(&uv);
            equal(source.x, uv.x.clone());
            equal(source.y, uv.y.clone());
            let actual = face
                .face
                .surface()
                .point_at(&{
                    let uv = &uv;
                    hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
                })
                .map(|point| {
                    point
                        .try_coordinates()
                        .expect("represented surface fixture")
                        .clone()
                })
                .unwrap();
            // Nonuniform weights map normalized t=1/2 to 2/3 on the parabola.
            let x = if kind == 1 { q(2, 3) } else { Real::zero() };
            equal(actual.x, &x * &scale);
            equal(actual.y, &x * &x * &scale);
            equal(actual.z, q(3, 2) * &scale);
            let options = StepWriteOptions::new("independent.step", "2026-10-18T00:00:00");
            let output = write_advanced(
                &[StepBody {
                    solid,
                    name: "independent",
                }],
                &options,
                &context(),
            )
            .result
            .unwrap()
            .unwrap();
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
            equal(
                again.bodies()[0].solid().volume().unwrap(),
                solid.volume().unwrap(),
            );
            replay(again.bodies()[0].model());
            let region = profile(false, false, false)
                .transform_affine(
                    &scale,
                    &Real::zero(),
                    &Real::zero(),
                    &scale,
                    &Real::zero(),
                    &Real::zero(),
                )
                .unwrap();
            let cutter =
                builder::extrude_curve_region(&region, scale.clone(), Real::from(2) * &scale)
                    .unwrap();
            let selected = BodySet::from_solid(solid);
            let middle = boolean::intersection(&selected, &cutter).unwrap();
            let ends = boolean::difference(&selected, &cutter).unwrap();
            let restored = boolean::union(&ends, &middle).unwrap();
            equal(
                restored.bodies().map(|s| s.volume().unwrap()).sum(),
                solid.volume().unwrap(),
            );
            assert!(
                boolean::difference(&selected, &restored)
                    .unwrap()
                    .is_empty()
            );
        }
    }
}

pub fn invalid_spline_records_and_exhausted_limits_never_publish_a_body() {
    for (controls, multiplicities) in [("#10,#10,#11,#12", "4,3"), ("#10,#11,#12,#12", "3,4")] {
        let record = format!(
            "B_SPLINE_CURVE_WITH_KNOTS('',2,({controls}),.UNSPECIFIED.,.F.,.U.,({multiplicities}),(2.,4.),.UNSPECIFIED.)"
        );
        let text = replace_record(INDEPENDENT, 30, &record);
        let error = import(&text, &ImportLimits::default()).unwrap_err();
        assert!(
            matches!(
                error.kind,
                ErrorKind::Geometry(GeometryError::InvalidKnotMultiplicity)
            ),
            "{error:?}"
        );
        assert_eq!(error.entity, Some(30));
    }
    let invalid = [
        INDEPENDENT.replace("'',2,(#10,#11,#12)", "'',0,(#10,#11,#12)"),
        INDEPENDENT.replace("(3,3),(2.,4.)", "(3,3000000000),(2.,4.)"),
        INDEPENDENT.replace("(3,3),(2.,4.)", "(3,3),(4.,2.)"),
        INDEPENDENT.replace("(3,3),(2.,4.)", "(3,3),(2.,2.)"),
        INDEPENDENT.replace(".F.,.U.,(3,3)", ".T.,.U.,(3,3)"),
        INDEPENDENT.replace(".F.,.U.,(3,3)", ".INVALID.,.U.,(3,3)"),
        replace_record(INDEPENDENT, 40, "EDGE_CURVE('',#20,#21,#30,.F.)"),
        replace_record(INDEPENDENT, 30, &rational_record(30, "1.,0.,1.")),
        replace_record(INDEPENDENT, 30, &rational_record(30, "1.,-2.,1.")),
        replace_record(INDEPENDENT, 30, &rational_record(30, "1.,1.")),
        replace_record(INDEPENDENT, 30, &rational_record(30, "1.,1.,1.,1.")),
        replace_record(INDEPENDENT, 20, "VERTEX_POINT('',#50)"),
        replace_record(INDEPENDENT, 37, "VECTOR('',#36,0.)"),
    ];
    for (index, text) in invalid.iter().enumerate() {
        let error = import(text, &ImportLimits::default()).unwrap_err();
        assert!(error.entity.is_some(), "case {index}: {error:?}");
        assert!(
            matches!(
                error.kind,
                ErrorKind::Invalid(_) | ErrorKind::Expected(_) | ErrorKind::Geometry(_)
            ),
            "case {index}: {error:?}"
        );
    }
    let error = import(
        INDEPENDENT,
        &ImportLimits {
            spline_knots: 5,
            ..ImportLimits::default()
        },
    )
    .unwrap_err();
    assert!(matches!(
        error.kind,
        ErrorKind::Limit {
            resource: Resource::SplineKnots,
            limit: 5,
            observed: 6
        }
    ));
    assert_eq!(error.entity, Some(30));
    // A separate operation after every rejection has no retained failed state.
    let imported = import(INDEPENDENT, &ImportLimits::default()).unwrap();
    equal(imported.bodies()[0].solid().volume().unwrap(), 32.into());
}

pub fn spline_whole_images_ambiguous_trims_and_supplied_pcurves_are_checked() {
    // The same endpoints do not prove equal interiors on the swept wall.
    let bowed_top = replace_record(INDEPENDENT, 14, "CARTESIAN_POINT('',(0.,-3.,3.))");
    let error = import(&bowed_top, &ImportLimits::default()).unwrap_err();
    assert!(
        matches!(error.kind, ErrorKind::Validation(_) | ErrorKind::Build(_)),
        "{error:?}"
    );
    let ambiguous = replace_record(
        INDEPENDENT,
        30,
        "B_SPLINE_CURVE_WITH_KNOTS('',2,(#10,#12,#10),.UNSPECIFIED.,.T.,.U.,(3,3),(2.,4.),.UNSPECIFIED.)",
    );
    let error = import(&ambiguous, &ImportLimits::default()).unwrap_err();
    assert!(
        matches!(
            error.kind,
            ErrorKind::Unsupported("ambiguous spline vertex preimage")
        ),
        "{error:?}"
    );
    for milli in [false, true] {
        // A dimensionless UV line on the extrusion, independently parameterized
        // as (2,0)+t*(1,0). Its chart must not acquire the model length scale.
        let mut text = INDEPENDENT.replace("ENDSEC;\nEND-ISO", "#200=PARAMETRIC_REPRESENTATION_CONTEXT('','');\n#201=CARTESIAN_POINT('',(2.,0.));\n#202=DIRECTION('',(1.,0.));\n#203=VECTOR('',#202,1.);\n#204=LINE('',#201,#203);\n#205=DEFINITIONAL_REPRESENTATION('',(#204),#200);\n#206=PCURVE('',#60,#205);\n#207=SURFACE_CURVE('',#30,(#206),.PCURVE_S1.);\nENDSEC;\nEND-ISO");
        text = replace_record(&text, 40, "EDGE_CURVE('',#20,#21,#207,.T.)");
        if milli {
            text = text.replace("SI_UNIT($,.METRE.)", "SI_UNIT(.MILLI.,.METRE.)");
        }
        let imported = import(&text, &ImportLimits::default()).unwrap();
        let body = &imported.bodies()[0];
        let edge = body
            .explicit_edges()
            .find(|edge| edge.source == 40)
            .unwrap();
        assert_eq!(edge.master, 206);
        assert_eq!(edge.geometry, 207);
        assert_eq!(edge.parameters.len(), 3);
        for parameters in edge.parameters {
            let (start, end) = if parameters.curve == 30 {
                (2, 4)
            } else {
                (0, 2)
            };
            equal(parameters.start.clone(), start.into());
            equal(parameters.end.clone(), end.into());
        }
        replay(body.model());
        let bad = replace_record(&text, 201, "CARTESIAN_POINT('',(2.,1.))");
        assert!(matches!(
            import(&bad, &ImportLimits::default()).unwrap_err().kind,
            ErrorKind::Build(_)
        ));
        // Success after a failed independent representation does not inherit it.
        import(&text, &ImportLimits::default()).unwrap();
    }
}

pub fn spline_write_limits_units_and_exact_numeric_failures_are_explicit() {
    let imported = import(INDEPENDENT, &ImportLimits::default()).unwrap();
    let selection = [StepBody {
        solid: imported.bodies()[0].solid(),
        name: "limits",
    }];
    let mut options = StepWriteOptions::new("limits.step", "2026-10-18T00:00:00");
    options.length_unit = WriteLengthUnit {
        name: "millimetre",
        metres_per_unit: q(1, 1000),
    };
    let out = write_advanced(&selection, &options, &context())
        .result
        .unwrap()
        .unwrap();
    let entities = StepFile::read(out.text().as_bytes(), &StepReadOptions::default()).unwrap();
    let round = entities
        .import_representation(out.representation(), &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    verify(round.bodies()[0].model(), false, false);
    for limit in 0..3 {
        let mut restricted = options.clone();
        match limit {
            0 => restricted.limits.entities = 10,
            1 => restricted.limits.output_bytes = 200,
            _ => restricted.limits.edge_uses = 1,
        }
        let error = write_advanced(&selection, &restricted, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(matches!(error.kind, WriteErrorKind::Limit { .. }));
    }
    assert_eq!(
        write_advanced(&selection, &options, &context())
            .result
            .unwrap()
            .unwrap()
            .text(),
        out.text()
    );
    // A rational magnitude with denominator 3 is exact internally but has no
    // finite decimal STEP encoding. Rejection does not replace it by a float.
    let map = Matrix4::affine_nonuniform_scale([q(1, 3), Real::one(), Real::one()]);
    let model = imported.bodies()[0].model().transformed(&map).unwrap();
    let error = write_advanced(
        &[StepBody {
            solid: model.solids().next().unwrap(),
            name: "third",
        }],
        &StepWriteOptions::new("third.step", "2026-10-18T00:00:00"),
        &context(),
    )
    .result
    .unwrap()
    .unwrap_err();
    assert!(matches!(error.kind, WriteErrorKind::NonTerminatingDecimal));
}
