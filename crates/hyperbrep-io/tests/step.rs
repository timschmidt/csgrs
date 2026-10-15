use hyperbrep::{
    ArchiveLimits, Direction, Matrix4, OperationContext, Point3, RawModel, Real,
    SolidPointLocation, WorkBudget,
};
use hyperbrep_io::step::{
    EdgeUseSource, ErrorKind, ImportLimits, Resource, StepFile, StepReadOptions, UnitKind,
    VertexSource,
};

const BOX: &str = include_str!("fixtures/faceted-box.step");
const TWO_BOXES: &str = include_str!("fixtures/faceted-two-boxes.step");
const THROUGH_HOLE: &str = include_str!("fixtures/faceted-through-hole.step");
const TETRAHEDRON: &str = include_str!("fixtures/faceted-tetrahedron.step");
fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    })
}
fn read(source: &str) -> StepFile<'_> {
    StepFile::read(source.as_bytes(), &StepReadOptions::default()).unwrap()
}
fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn faceted_boxes_map_exact_units_shared_edges_and_source_ownership() {
    for (source, count) in [(BOX, 1), (TWO_BOXES, 2)] {
        let file = read(source);
        let imported = file
            .import_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        assert_eq!(imported.bodies().len(), count);
        assert_eq!(imported.units().length.scale_to_si, ratio(1, 1000));
        assert_eq!(
            imported.units().uncertainties[0].value_in_si,
            ratio(1, 1_000_000_000) * ratio(1, 10)
        );
        for (index, body) in imported.bodies().iter().enumerate() {
            let counts = body.model().counts();
            assert_eq!(
                (
                    counts.vertices,
                    counts.edges,
                    counts.edge_uses,
                    counts.faces,
                    counts.shells,
                    counts.solids
                ),
                (8, 12, 24, 6, 1, 1)
            );
            assert_eq!(body.solid().volume().unwrap(), ratio(3, 125_000_000));
            assert_eq!(body.faces().count(), 6);
            assert_eq!(body.vertices().count(), 8);
            assert_eq!(body.loops().count(), 6);
            let point = Point3::new(
                ratio(1 + 10 * index as i32, 1000),
                ratio(1, 1000),
                ratio(1, 1000),
            );
            assert_eq!(
                body.solid().classify_point(&point).unwrap(),
                SolidPointLocation::Inside
            );
            let json = body.model().to_json().unwrap();
            let replayed = RawModel::from_json(&json, &ArchiveLimits::default())
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(replayed.to_json().unwrap(), json);
            let moved = replayed
                .transformed(&Matrix4::affine_translation([
                    Real::one(),
                    Real::zero(),
                    Real::zero(),
                ]))
                .unwrap();
            let solid = moved.solids().next().unwrap();
            assert_eq!(solid.volume().unwrap(), body.solid().volume().unwrap());
            assert_eq!(
                solid
                    .classify_point(&Point3::new(
                        &point.x + Real::one(),
                        point.y.clone(),
                        point.z.clone()
                    ))
                    .unwrap(),
                SolidPointLocation::Inside
            );
            let uses = body.edge_uses().collect::<Vec<_>>();
            assert_eq!(uses.len(), counts.edge_uses);
            for (i, item) in uses.iter().enumerate() {
                let EdgeUseSource::PolygonSegment {
                    from_point,
                    to_point,
                    ..
                } = item.source
                else {
                    panic!("polygon provenance");
                };
                assert!(
                    !uses[..i]
                        .iter()
                        .any(|other| other.edge_use == item.edge_use)
                );
                let edge = item.edge_use.edge();
                let (from, to) = match item.edge_use.direction() {
                    Direction::Forward => (edge.start(), edge.end()),
                    Direction::Reversed => (edge.end(), edge.start()),
                };
                assert_eq!(
                    body.vertices()
                        .find(|v| v.source == VertexSource::CartesianPoint { point: from_point })
                        .unwrap()
                        .vertex,
                    from
                );
                assert_eq!(
                    body.vertices()
                        .find(|v| v.source == VertexSource::CartesianPoint { point: to_point })
                        .unwrap()
                        .vertex,
                    to
                );
                assert_eq!(edge.uses().count(), 2);
                assert_eq!(
                    body.loops()
                        .find(|(bound, _, _)| *bound == item.bound)
                        .unwrap()
                        .2,
                    item.edge_use.wire()
                );
            }
        }
        assert!(imported.retained_entities().contains(&9503));
        assert!(imported.interpreted_entities().contains(&9000));
        assert_eq!(
            imported.interpreted_entities().len() + imported.retained_entities().len(),
            file.syntax().entities().len()
        );
    }
}

#[test]
fn exact_coordinate_aliases_share_vertices_without_losing_source_ids() {
    let source = BOX.replace(
        "#104=POLY_LOOP('',(#1,#5,#8,#4));",
        "#104=POLY_LOOP('',(#9,#5,#8,#4));\n#9=CARTESIAN_POINT('alias',(0.E3,+0.0,-0.));",
    );
    let file = read(&source);
    let imported = file
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    let body = &imported.bodies()[0];
    assert_eq!(body.model().counts().vertices, 8);
    let vertices = body.vertices().collect::<Vec<_>>();
    assert_eq!(vertices.len(), 9);
    assert_eq!(
        vertices
            .iter()
            .find(|v| v.source == VertexSource::CartesianPoint { point: 1 })
            .unwrap()
            .vertex,
        vertices
            .iter()
            .find(|v| v.source == VertexSource::CartesianPoint { point: 9 })
            .unwrap()
            .vertex
    );
}

#[test]
fn reversed_bound_preserves_the_authored_face_and_bad_sense_is_rejected() {
    let reversed = BOX
        .replace(
            "#104=POLY_LOOP('',(#1,#5,#8,#4));",
            "#104=POLY_LOOP('',(#4,#8,#5,#1));",
        )
        .replace(
            "#105=FACE_OUTER_BOUND('',#104,.T.);",
            "#105=FACE_OUTER_BOUND('',#104,.F.);",
        );
    let file = read(&reversed);
    let imported = file
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(
        imported.bodies()[0]
            .edge_uses()
            .filter(|u| u.loop_id == 104)
            .map(|u| match u.source {
                EdgeUseSource::PolygonSegment {
                    segment,
                    from_point,
                    to_point,
                } => (segment, from_point, to_point),
                _ => panic!("polygon provenance"),
            })
            .collect::<Vec<_>>(),
        vec![(2, 1, 5), (1, 5, 8), (0, 8, 4), (3, 4, 1)]
    );
    let wrong = BOX.replace(
        "#106=FACE_SURFACE('',(#105),#103,.T.);",
        "#106=FACE_SURFACE('',(#105),#103,.F.);",
    );
    let file = read(&wrong);
    let error = file
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap_err();
    assert_eq!(error.entity, Some(106));
    assert!(matches!(error.kind, ErrorKind::Build(_)));
    let consistent = wrong.replace(
        "#100=DIRECTION('',(-1.,0.,0.));",
        "#100=DIRECTION('',(1.,0.,0.));",
    );
    let consistent = read(&consistent)
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(
        consistent.bodies()[0].solid().volume().unwrap(),
        ratio(24, 1_000_000_000)
    );
}

#[test]
fn import_limits_and_unsupported_items_never_publish_partial_bodies() {
    for (limits, resource) in [
        (
            ImportLimits {
                entity_visits: 0,
                ..ImportLimits::default()
            },
            Resource::EntityVisits,
        ),
        (
            ImportLimits {
                scalars: 0,
                ..ImportLimits::default()
            },
            Resource::Scalars,
        ),
        (
            ImportLimits {
                text_bytes: 0,
                ..ImportLimits::default()
            },
            Resource::DecodedTextBytes,
        ),
        (
            ImportLimits {
                edge_uses: 23,
                ..ImportLimits::default()
            },
            Resource::EdgeUses,
        ),
    ] {
        let file = read(BOX);
        let error = file
            .import_representation(502, &limits, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(matches!(error.kind,ErrorKind::Limit{resource:r,..} if r==resource));
    }
    let source = BOX.replace("('boxes',(#501)", "('boxes',(#501,#9503)");
    let file = read(&source);
    let error = file
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap_err();
    assert_eq!(error.entity, Some(9503));
    assert!(matches!(error.kind, ErrorKind::Unsupported(_)));
}

#[test]
fn conversion_units_use_declared_factors_and_detect_cycles_and_dimensions() {
    let source=BOX.replace("#9000=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));",
        "#9000=(CONVERSION_BASED_UNIT('inch',#9011)LENGTH_UNIT()NAMED_UNIT(#9012));\n#9011=MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9010);\n#9012=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\n#9010=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));");
    let file = read(&source);
    let units = file.context_units(9003, &ImportLimits::default()).unwrap();
    assert_eq!(units.length.scale_to_si, ratio(127, 5000));
    assert_eq!(units.length.kind, UnitKind::Length);
    let declared = source.replace("LENGTH_MEASURE(25.4)", "LENGTH_MEASURE(20.)");
    assert_eq!(
        read(&declared)
            .context_units(9003, &ImportLimits::default())
            .unwrap()
            .length
            .scale_to_si,
        ratio(1, 50)
    );
    let cyclic = source.replace("LENGTH_MEASURE(25.4),#9010", "LENGTH_MEASURE(25.4),#9000");
    assert!(matches!(
        read(&cyclic)
            .context_units(9003, &ImportLimits::default())
            .unwrap_err()
            .kind,
        ErrorKind::UnitCycle(9000)
    ));
    let wrong = source.replace("DIMENSIONAL_EXPONENTS(1.,0.", "DIMENSIONAL_EXPONENTS(0.,1.");
    assert!(matches!(
        read(&wrong)
            .context_units(9003, &ImportLimits::default())
            .unwrap_err()
            .kind,
        ErrorKind::Invalid(_)
    ));
}

#[test]
fn header_decoding_retains_original_tokens_and_rejects_invalid_text() {
    let source = BOX.replace(
        "'boxes.step'",
        r"'one''two\\\X2\03A9\X0\\X4\0001F642\X0\\PA\\S\d\X\0A'",
    );
    let file = read(&source);
    assert_eq!(file.header().name, "one'two\\Ω🙂ä\n");
    assert_eq!(file.syntax().source(), source);
    for token in [
        r"'\X2\D800\X0\'",
        r"'\X4\00110000\X0\'",
        r"'\X2\\X0\'",
        r"'\X2\03π\X0\'",
        r"'\S\π'",
        r"'\PB\\S\d'",
    ] {
        let bad = BOX.replace("'boxes.step'", token);
        assert!(
            StepFile::read(bad.as_bytes(), &StepReadOptions::default()).is_err(),
            "{token}"
        );
    }
}

#[test]
fn inner_bounds_and_oblique_planes_construct_complete_exact_solids() {
    for (source, vertices, edges, faces, volume) in
        [(THROUGH_HOLE, 16, 24, 10, 20), (TETRAHEDRON, 4, 6, 4, 12)]
    {
        let file = read(source);
        let imported = file
            .import_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        let body = &imported.bodies()[0];
        let counts = body.model().counts();
        assert_eq!(
            (counts.vertices, counts.edges, counts.faces),
            (vertices, edges, faces)
        );
        assert_eq!(body.solid().volume().unwrap(), ratio(volume, 1_000_000_000));
    }
    let file = read(THROUGH_HOLE);
    let imported = file
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    let body = &imported.bodies()[0];
    assert_eq!(
        body.faces()
            .filter(|face| face.face.inner().count() == 1)
            .count(),
        2
    );
    assert_eq!(
        body.solid()
            .classify_point(&Point3::new(ratio(1, 1000), ratio(3, 2000), ratio(2, 1000)))
            .unwrap(),
        SolidPointLocation::Outside
    );
}

#[test]
fn placement_defaults_projection_and_metadata_have_step_semantics() {
    for (old, new) in [
        (
            "#102=AXIS2_PLACEMENT_3D('',#1,#100,#101);",
            "#102=AXIS2_PLACEMENT_3D('',#1,#100,$);",
        ),
        (
            "#152=AXIS2_PLACEMENT_3D('',#5,#150,#151);",
            "#152=AXIS2_PLACEMENT_3D('',#5,$,$);",
        ),
        (
            "#101=DIRECTION('',(0.,0.,1.));",
            "#101=DIRECTION('',(3.,0.,4.));",
        ),
    ] {
        let source = BOX.replace(old, new);
        let imported = read(&source)
            .import_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        assert_eq!(
            imported.bodies()[0].solid().volume().unwrap(),
            ratio(24, 1_000_000_000)
        );
    }
    let source = BOX.replace("('boxes',(#501)", "('boxes',(#501,#152)");
    let imported = read(&source)
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(imported.placement_items(), &[152]);
    assert_eq!(
        imported.bodies()[0]
            .solid()
            .classify_point(&Point3::new(ratio(1, 1000), ratio(1, 1000), ratio(1, 1000)))
            .unwrap(),
        SolidPointLocation::Inside
    );
    for replacement in ["(0.,0.,0.)", "(-1.,0.,0.)"] {
        let source = BOX.replace(
            "#101=DIRECTION('',(0.,0.,1.));",
            &format!("#101=DIRECTION('',{replacement});"),
        );
        let error = read(&source)
            .import_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(matches!(error.kind, ErrorKind::Numeric(_)));
    }
}

#[test]
fn authored_uncertainty_never_closes_an_exact_gap() {
    let source = BOX.replace(
        "#104=POLY_LOOP('',(#1,#5,#8,#4));",
        "#104=POLY_LOOP('',(#9,#5,#8,#4));\n#9=CARTESIAN_POINT('distinct',(1.E-12,0.,0.));",
    );
    let error = read(&source)
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap_err();
    assert!(matches!(
        error.kind,
        ErrorKind::Build(_) | ErrorKind::Validation(_)
    ));
    // The declared uncertainty is 1e-7 mm; this 1e-12 mm discrepancy stays real.
    assert_eq!(
        read(&source)
            .context_units(9003, &ImportLimits::default())
            .unwrap()
            .uncertainties[0]
            .value_in_si,
        ratio(1, 1_000_000_000) * ratio(1, 10)
    );
    let ambient = hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, || {
        read(&source)
            .import_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap_err()
    });
    assert!(matches!(
        ambient.certified().unwrap().kind,
        ErrorKind::Build(_) | ErrorKind::Validation(_)
    ));
}

#[test]
fn geometry_above_binary64_integer_precision_keeps_its_exact_extent() {
    let source = BOX
        .replace("POINT('',(0.,", "POINT('',(9007199254740993.,")
        .replace("POINT('',(2.,", "POINT('',(9007199254740995.,");
    let imported = read(&source)
        .import_representation(502, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    let body = &imported.bodies()[0];
    assert_eq!(body.solid().volume().unwrap(), ratio(24, 1_000_000_000));
    let point = Point3::new(
        Real::from(9_007_199_254_740_994_u64) * ratio(1, 1000),
        ratio(1, 1000),
        ratio(1, 1000),
    );
    assert_eq!(
        body.solid().classify_point(&point).unwrap(),
        SolidPointLocation::Inside
    );
}
