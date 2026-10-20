use hyperbrep::{OperationContext, WorkBudget};
use hyperbrep_io::step::{Error, ErrorKind, ImportLimits, Resource, StepFile, StepReadOptions};

const BOX: &str = include_str!("fixtures/faceted-box.step");
fn rejected(source: &str) -> Error {
    StepFile::read(source.as_bytes(), &StepReadOptions::default())
        .unwrap()
        .import_representation(
            502,
            &ImportLimits::default(),
            &OperationContext::strict(WorkBudget {
                refinements: 10_000,
                promotions: 1000,
            }),
        )
        .result
        .unwrap()
        .unwrap_err()
}

#[test]
fn wrong_topology_and_attribute_types_report_source_entities() {
    for (old, new, entity) in [
        ("('boxes',(#501)", "('boxes',(#501,#501)", 502),
        (
            "CLOSED_SHELL('',(#106,#116",
            "CLOSED_SHELL('',(#106,#106,#116",
            500,
        ),
        ("FACE_SURFACE('',(#105)", "FACE_SURFACE('',(#105,#105)", 106),
        (
            "POLY_LOOP('',(#1,#5,#8,#4))",
            "POLY_LOOP('',(#1,#5,#8,#1))",
            104,
        ),
        ("POLY_LOOP('',(#1,#5,#8,#4))", "POLY_LOOP('',(#1,#5))", 104),
        (
            "#105=FACE_OUTER_BOUND('',#104,.T.)",
            "#105=FACE_OUTER_BOUND('',#104,.U.)",
            105,
        ),
        (
            "#105=FACE_OUTER_BOUND('',#104,.T.)",
            "#105=FACE_BOUND('',#104,.T.)",
            106,
        ),
        (
            "#1=CARTESIAN_POINT('',(0.,0.,0.))",
            "#1=CARTESIAN_POINT('',(0.,0.))",
            1,
        ),
        (
            "#1=CARTESIAN_POINT('',(0.,0.,0.))",
            "#1=CARTESIAN_POINT('',(0.,0.,$))",
            1,
        ),
        (
            "#1=CARTESIAN_POINT('',(0.,0.,0.))",
            "#1=CARTESIAN_POINT('',(0.,0.,0.),'extra')",
            1,
        ),
        (
            "GEOMETRIC_REPRESENTATION_CONTEXT(3)",
            "GEOMETRIC_REPRESENTATION_CONTEXT(3.)",
            9003,
        ),
        (
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#9004))",
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(())",
            9003,
        ),
        (
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#9004))",
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#9004,#9004))",
            9003,
        ),
        ("LENGTH_MEASURE(1.E-7)", "LENGTH_MEASURE(-1.E-7)", 9004),
    ] {
        let changed = BOX.replace(old, new);
        assert_ne!(changed, BOX);
        let error = rejected(&changed);
        assert_eq!(error.entity, Some(entity), "{error:?}");
        assert!(
            matches!(error.kind, ErrorKind::Invalid(_) | ErrorKind::Expected(_)),
            "{error:?}"
        );
        assert!(error.offset > 0);
    }
    let missing = BOX.replace(
        "#104=POLY_LOOP('',(#1,#5,#8,#4));",
        "#104=POLY_LOOP('',(#99999,#5,#8,#4));",
    );
    let error = rejected(&missing);
    assert_eq!(error.entity, Some(104));
    assert_eq!(&missing[error.offset..error.offset + 6], "#99999");
    assert!(matches!(error.kind, ErrorKind::MissingEntity(99999)));
}

#[test]
fn header_admission_and_schema_scope_are_explicit() {
    for source in [
        BOX.replace(
            "FILE_DESCRIPTION(('Independently authored polygonal boxes'),'2;1');",
            "",
        ),
        BOX.replace("('HyperBRep tests')", "()"),
        BOX.replace("'2;1'", "'2;2'"),
        BOX.replace(
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));",
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));",
        ),
        BOX.replace(
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));",
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN','AUTOMOTIVE_DESIGN'));",
        ),
        BOX.replace("DATA;", "DATA('part',('UNDECLARED'));"),
    ] {
        assert!(StepFile::read(source.as_bytes(), &StepReadOptions::default()).is_err());
    }
    let error = StepFile::read(
        BOX.as_bytes(),
        &StepReadOptions {
            header_text_bytes: 0,
            ..StepReadOptions::default()
        },
    )
    .unwrap_err();
    assert!(matches!(
        error.kind,
        ErrorKind::Limit {
            resource: Resource::DecodedTextBytes,
            ..
        }
    ));
    let named = BOX.replace("DATA;", "DATA('part',('AUTOMOTIVE_DESIGN'));");
    assert!(StepFile::read(named.as_bytes(), &StepReadOptions::default()).is_ok());
    let unknown = BOX.replace("'AUTOMOTIVE_DESIGN'", "'UNKNOWN_SCHEMA'");
    assert!(matches!(rejected(&unknown).kind, ErrorKind::Unsupported(_)));
    for schema in [
        "CONFIG_CONTROL_DESIGN",
        "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF",
    ] {
        let source = BOX.replace("AUTOMOTIVE_DESIGN", schema);
        assert!(
            StepFile::read(source.as_bytes(), &StepReadOptions::default())
                .unwrap()
                .import_representation(
                    502,
                    &ImportLimits::default(),
                    &OperationContext::strict(WorkBudget {
                        refinements: 10_000,
                        promotions: 1000
                    })
                )
                .result
                .unwrap()
                .is_ok()
        );
    }
}

#[test]
fn physical_controls_inside_text_directives_and_print_layout_do_not_change_content() {
    for (token, expected) in [
        (r"\X2\03A9\X0\", "Ω"),
        (r"\X4\0001F642\X0\", "🙂"),
        (r"\PA\\S\d", "ä"),
        (r"\X\0A", "\n"),
        (r"\N\\F\", ""),
    ] {
        let physical = token
            .chars()
            .map(|character| format!("{character}\r\n\t"))
            .collect::<String>();
        let source = BOX.replace("'boxes.step'", &format!("'a{physical}b'"));
        let file = StepFile::read(source.as_bytes(), &StepReadOptions::default()).unwrap();
        assert_eq!(file.header().name, format!("a{expected}b"));
        assert_eq!(file.syntax().source(), source);
    }
    for token in [
        r"\X2\123\X0\",
        r"\X4\FFFFFFFF\X0\",
        r"\X2\D800\X0\",
        r"\PJ\",
        r"\N",
    ] {
        let source = BOX.replace("'boxes.step'", &format!("'{token}'"));
        assert!(StepFile::read(source.as_bytes(), &StepReadOptions::default()).is_err());
    }
}
