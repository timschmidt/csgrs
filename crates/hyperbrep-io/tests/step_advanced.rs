use hyperbrep::{ArchiveLimits, Direction, OperationContext, RawModel, Real, WorkBudget};
use hyperbrep_io::step::{
    EdgeUseSource, ErrorKind, ImportLimits, Resource, StepFile, StepReadOptions, VertexSource,
};

const BOX: &str = include_str!("fixtures/occt-advanced-box.step");
const BOX_3D: &str = include_str!("fixtures/occt-advanced-box-3d.step");
const HOLE: &str = include_str!("fixtures/occt-advanced-hole.step");
fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    })
}
fn read(text: &str) -> StepFile<'_> {
    StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap()
}
fn argument(text: &str, entity: u64, index: usize, value: &str) -> String {
    let file = read(text);
    let span = file
        .syntax()
        .entity(entity)
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
fn append(text: &str, entity: &str) -> String {
    let mut result = text.to_string();
    result.insert_str(result.rfind("ENDSEC;").unwrap(), entity);
    result
}

#[test]
fn independent_advanced_boxes_retain_explicit_topology_and_all_curve_parameters() {
    for (text, maps) in [(BOX, 4), (BOX_3D, 1)] {
        let file = read(text);
        let imported = file
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        assert_eq!(imported.bodies().len(), 1);
        let body = &imported.bodies()[0];
        assert_eq!(
            body.solid().volume().unwrap(),
            (Real::from(24) / Real::from(1_000_000_000)).unwrap()
        );
        let counts = body.model().counts();
        assert_eq!(
            (
                counts.vertices,
                counts.edges,
                counts.edge_uses,
                counts.faces
            ),
            (8, 12, 24, 6)
        );
        assert_eq!(body.explicit_edges().count(), 12);
        for edge in body.explicit_edges() {
            assert_eq!(edge.parameters.len(), maps);
            assert!(edge.parameters.iter().any(|p| p.curve == edge.geometry));
            assert!(edge.parameters.iter().any(|p| p.curve == edge.master));
            assert_eq!(edge.edge.uses().count(), 2);
        }
        for item in body.edge_uses() {
            let EdgeUseSource::OrientedEdge {
                edge_curve,
                from_vertex,
                to_vertex,
                ..
            } = item.source
            else {
                panic!("explicit edge source");
            };
            let edge = item.edge_use.edge();
            assert_eq!(
                body.explicit_edges()
                    .find(|x| x.source == edge_curve)
                    .unwrap()
                    .edge,
                edge
            );
            let (from, to) = if item.edge_use.direction() == Direction::Forward {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            let vertex = |id| {
                body.vertices()
                    .find(|v| matches!(v.source,VertexSource::VertexPoint{vertex,..} if vertex==id))
                    .unwrap()
                    .vertex
            };
            assert_eq!(vertex(from_vertex), from);
            assert_eq!(vertex(to_vertex), to);
        }
        let json = body.model().to_json().unwrap();
        let loaded = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(loaded.to_json().unwrap(), json);
        assert_eq!(
            imported.interpreted_entities().len() + imported.retained_entities().len(),
            file.syntax().entities().len()
        );
    }
}

#[test]
fn independent_hole_bounds_can_precede_their_outer_loop_and_survive_edits() {
    let file = read(HOLE);
    let imported = file
        .import_representation(10, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    let body = &imported.bodies()[0];
    let volume = (Real::from(20) / Real::from(1_000_000_000)).unwrap();
    assert_eq!(body.solid().volume().unwrap(), volume);
    assert_eq!(
        (
            body.model().counts().vertices,
            body.model().counts().edges,
            body.model().counts().faces
        ),
        (16, 24, 10)
    );
    let mut reordered = HOLE.to_string();
    let mut holes = 0;
    for face in body.faces().filter(|f| f.face.inner().count() == 1) {
        let source = file
            .syntax()
            .entity(face.source)
            .unwrap()
            .components()
            .next()
            .unwrap()
            .arguments()
            .nth(1)
            .unwrap();
        let ids = source
            .children()
            .map(|v| match v.kind() {
                hyperbrep_io::part21::ValueKind::Reference(id) => id,
                _ => panic!("bound id"),
            })
            .collect::<Vec<_>>();
        assert_eq!(ids.len(), 2);
        reordered = argument(
            &reordered,
            face.source,
            1,
            &format!("(#{},#{})", ids[1], ids[0]),
        );
        holes += 1;
    }
    assert_eq!(holes, 2);
    let reordered = read(&reordered)
        .import_representation(10, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(reordered.bodies()[0].solid().volume().unwrap(), volume);
    let split = body
        .model()
        .edges()
        .next()
        .unwrap()
        .split((Real::one() / Real::from(2)).unwrap().into())
        .unwrap()
        .into_model();
    let transformed = split
        .transformed(&hyperbrep::Matrix4::affine_nonuniform_scale([
            -Real::one(),
            Real::one(),
            Real::one(),
        ]))
        .unwrap();
    let json = transformed.to_json().unwrap();
    let replay = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.solids().next().unwrap().volume().unwrap(), volume);
}

#[test]
fn differing_surface_curve_parameters_keep_exact_master_and_alternate_maps() {
    let reparameterized = argument(&argument(BOX, 39, 1, "(7.,0.)"), 40, 2, "2.");
    for master in [".PCURVE_S1.", ".PCURVE_S2.", ".CURVE_3D."] {
        let text = argument(&reparameterized, 26, 3, master);
        let imported = read(&text)
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        let edge = imported.bodies()[0]
            .explicit_edges()
            .find(|e| e.source == 21)
            .unwrap();
        let map = |id| edge.parameters.iter().find(|p| p.curve == id).unwrap();
        assert_eq!(map(27).start, Real::zero());
        assert_eq!(map(27).end, Real::from(4));
        assert_eq!(map(31).start, (Real::from(-7) / Real::from(2)).unwrap());
        assert_eq!(map(31).end, (Real::from(-3) / Real::from(2)).unwrap());
        let chosen = match master {
            ".PCURVE_S1." => 31,
            ".PCURVE_S2." => 43,
            _ => 27,
        };
        assert_eq!(edge.master, chosen);
        assert_eq!(map(26).start, map(chosen).start);
        assert_eq!(map(26).end, map(chosen).end);
        assert_eq!(
            imported.bodies()[0].solid().volume().unwrap(),
            (Real::from(24) / Real::from(1_000_000_000)).unwrap()
        );
    }
    let reverse = argument(
        &argument(&argument(BOX, 21, 1, "#24"), 21, 2, "#22"),
        21,
        4,
        ".F.",
    );
    // Locate the other oriented use by source EDGE_CURVE identity, independent of numbering.
    let file = read(BOX);
    let mut reverse = reverse;
    for entity in file.syntax().entities() {
        let record = entity.components().next().unwrap();
        if record.name() != "ORIENTED_EDGE" {
            continue;
        }
        let values = record.arguments().collect::<Vec<_>>();
        if values[3].kind() == hyperbrep_io::part21::ValueKind::Reference(21) {
            let flag = if values[4].kind() == hyperbrep_io::part21::ValueKind::Enumeration("T") {
                ".F."
            } else {
                ".T."
            };
            reverse = argument(&reverse, entity.id(), 4, flag);
        }
    }
    let imported = read(&reverse)
        .import_representation(10, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    let edge = imported.bodies()[0]
        .explicit_edges()
        .find(|e| e.source == 21)
        .unwrap();
    assert!(
        edge.parameters
            .iter()
            .all(|p| p.start == Real::from(4) && p.end == Real::zero())
    );
}

#[test]
fn every_surface_curve_representation_and_explicit_topological_identity_is_checked() {
    let duplicate_vertex = append(
        &argument(BOX, 21, 1, "#9000"),
        "#9000=VERTEX_POINT('distinct',#23);\n",
    );
    let duplicate_edge = append(
        &argument(BOX, 20, 3, "#9001"),
        "#9001=EDGE_CURVE('distinct',#22,#24,#26,.T.);\n",
    );
    for text in [
        argument(BOX, 51, 1, "(0.,1.)"),  // inconsistent non-master pcurve
        argument(BOX, 41, 1, "(-1.,0.)"), // same set, opposite pcurve direction
        argument(BOX, 53, 1, "(-1.,0.)"), // opposite sense in the non-master pcurve
        argument(BOX, 29, 2, "0."),
        argument(BOX, 21, 4, ".F."), // edge direction disagrees with geometry
        argument(BOX, 20, 1, "#22"), // derived attribute must be *
        argument(BOX, 26, 2, "(#31,#43,#31)"),
        argument(BOX, 26, 2, "()"),
        argument(&argument(BOX, 26, 2, "(#31)"), 26, 3, ".PCURVE_S2."),
        argument(BOX, 26, 2, "(#32,#43)"), // ADVANCED_FACE requires PCURVEs
        duplicate_vertex,
        duplicate_edge,
    ] {
        let result = read(&text)
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .unwrap();
        assert!(
            result.is_err(),
            "accepted invalid advanced topology or geometry"
        );
    }
}

#[test]
fn exact_translation_nonunit_directions_and_parametric_context_forms_are_preserved() {
    let mut translated = String::new();
    for line in BOX.lines() {
        let line = if line.contains("CARTESIAN_POINT") && line.matches(',').count() == 3 {
            line.replace("('',(0.,", "('',(9007199254740993.,")
                .replace("('',(2.,", "('',(9007199254740995.,")
        } else {
            line.to_string()
        };
        translated.push_str(&line);
        translated.push('\n');
    }
    let nonunit = argument(&argument(BOX, 30, 1, "(0.,0.,5.)"), 29, 2, "2.");
    let file = read(BOX);
    let span = file.syntax().entity(42).unwrap().span();
    for text in [
        translated,
        nonunit,
        {
            let mut text = BOX.to_string();
            text.replace_range(
                span.start..span.end,
                "#42=PARAMETRIC_REPRESENTATION_CONTEXT('uv','');",
            );
            text
        },
        {
            let mut text = BOX.to_string();
            text.replace_range(
                span.start..span.end,
                "#42=(PARAMETRIC_REPRESENTATION_CONTEXT()REPRESENTATION_CONTEXT('uv',''));",
            );
            text
        },
    ] {
        let imported = read(&text)
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        assert_eq!(
            imported.bodies()[0].solid().volume().unwrap(),
            (Real::from(24) / Real::from(1_000_000_000)).unwrap()
        );
    }
}

#[test]
fn uncertainty_and_ambient_approximation_never_repair_an_off_line_vertex() {
    let gap = argument(BOX, 23, 1, "(1.E-12,0.,0.)");
    let import = || {
        read(&gap)
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap_err()
    };
    let error = import();
    // The reversed first bound visits edge #56 before edge #21; both use #22.
    assert_eq!(error.entity, Some(56));
    assert!(matches!(error.kind, ErrorKind::Invalid(_)));
    let ambient = hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, import);
    assert!(matches!(
        ambient.certified().unwrap().kind,
        ErrorKind::Invalid(_)
    ));
    for text in [
        BOX.replace(
            "GEOMETRIC_REPRESENTATION_CONTEXT(2)",
            "GEOMETRIC_REPRESENTATION_CONTEXT(3)",
        ),
        BOX.replace(
            "GEOMETRIC_REPRESENTATION_CONTEXT(2)",
            "GEOMETRIC_REPRESENTATION_CONTEXT(2.)",
        ),
        argument(BOX, 37, 1, "(#38,#38)"),
        argument(BOX, 39, 1, "(0.,0.,0.)"),
    ] {
        assert!(
            read(&text)
                .import_representation(10, &ImportLimits::default(), &context())
                .result
                .unwrap()
                .is_err()
        );
    }
}

#[test]
fn advanced_units_body_ownership_and_limits_are_explicit() {
    let metre = BOX.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)");
    let imported = read(&metre)
        .import_representation(10, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(
        imported.bodies()[0].solid().volume().unwrap(),
        Real::from(24)
    );
    let two = append(
        &argument(BOX, 10, 1, "(#11,#15,#9000)"),
        "#9000=MANIFOLD_SOLID_BREP('second',#16);\n",
    );
    let imported = read(&two)
        .import_representation(10, &ImportLimits::default(), &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(imported.bodies().len(), 2);
    assert_ne!(
        imported.bodies()[0].vertices().next().unwrap().vertex,
        imported.bodies()[1].vertices().next().unwrap().vertex
    );
    for (limits, resource) in [
        (
            ImportLimits {
                edge_uses: 23,
                ..ImportLimits::default()
            },
            Resource::EdgeUses,
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
                entity_visits: 0,
                ..ImportLimits::default()
            },
            Resource::EntityVisits,
        ),
        (
            ImportLimits {
                text_bytes: 0,
                ..ImportLimits::default()
            },
            Resource::DecodedTextBytes,
        ),
    ] {
        let error = read(BOX)
            .import_representation(10, &limits, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(matches!(error.kind,ErrorKind::Limit{resource:r,..} if r==resource));
    }
    assert!(
        read(BOX)
            .import_representation(
                10,
                &ImportLimits {
                    edge_uses: 24,
                    ..ImportLimits::default()
                },
                &context()
            )
            .result
            .unwrap()
            .is_ok()
    );
    let bad = argument(&two, 9000, 1, "#999999");
    assert!(
        read(&bad)
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .is_err()
    );
}
