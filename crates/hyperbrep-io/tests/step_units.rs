mod support;
use hyperbrep_io::step::{ErrorKind, ImportLimits, Resource, StepFile, StepReadOptions, UnitKind};
use hyperreal::Real;

fn units(
    source: &str,
    limits: &ImportLimits,
) -> Result<hyperbrep_io::step::ContextUnits, hyperbrep_io::step::Error> {
    StepFile::read(source.as_bytes(), &StepReadOptions::default())
        .unwrap()
        .context_units(1, limits)
}

#[test]
fn deep_conversion_graph_uses_bounded_native_stack_and_detects_back_edges() {
    let source = support::unit_chain(20_000, "1.");
    std::thread::Builder::new()
        .stack_size(64 * 1024)
        .spawn(move || {
            assert_eq!(
                units(&source, &ImportLimits::default())
                    .unwrap()
                    .length
                    .scale_to_si,
                Real::one()
            );
            let cyclic = source.replace("LENGTH_MEASURE(1.),#3);", "LENGTH_MEASURE(1.),#100);");
            assert!(matches!(
                units(&cyclic, &ImportLimits::default()).unwrap_err().kind,
                ErrorKind::UnitCycle(100)
            ));
            assert!(matches!(
                units(
                    &source,
                    &ImportLimits {
                        entity_visits: 100,
                        ..ImportLimits::default()
                    }
                )
                .unwrap_err()
                .kind,
                ErrorKind::Limit {
                    resource: Resource::EntityVisits,
                    ..
                }
            ));
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn resolved_scale_growth_is_admitted_before_multiplication() {
    let source = support::unit_chain(4, "2.");
    let resolved = units(
        &source,
        &ImportLimits {
            unit_scale_bits: 6,
            ..ImportLimits::default()
        },
    )
    .unwrap();
    assert_eq!(resolved.length.scale_to_si, Real::from(16));
    let file = StepFile::read(source.as_bytes(), &StepReadOptions::default()).unwrap();
    let error = file
        .context_units(
            1,
            &ImportLimits {
                unit_scale_bits: 5,
                ..ImportLimits::default()
            },
        )
        .unwrap_err();
    assert!(matches!(
        error.kind,
        ErrorKind::Limit {
            resource: Resource::UnitScaleBits,
            limit: 5,
            observed: 6
        }
    ));
    assert_eq!(error.entity, Some(100));
    // Inclusive SI boundary, and a conversion product bound conservative about cancellation.
    let si = support::unit_chain(0, "1.");
    assert_eq!(
        units(
            &si,
            &ImportLimits {
                unit_scale_bits: 1,
                ..ImportLimits::default()
            }
        )
        .unwrap()
        .length
        .scale_to_si,
        Real::one()
    );
    assert!(matches!(
        units(
            &si,
            &ImportLimits {
                unit_scale_bits: 0,
                ..ImportLimits::default()
            }
        )
        .unwrap_err()
        .kind,
        ErrorKind::Limit {
            resource: Resource::UnitScaleBits,
            ..
        }
    ));
}

#[test]
fn measure_variants_and_invalid_graphs_have_typed_results() {
    let source = support::unit_chain(1, "25.4");
    for replacement in [
        "#101=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#3);",
        "#101=(LENGTH_MEASURE_WITH_UNIT()MEASURE_WITH_UNIT(POSITIVE_LENGTH_MEASURE(25.4),#3));",
    ] {
        let source = source.replace(
            "#101=MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#3);",
            replacement,
        );
        assert_eq!(
            units(&source, &ImportLimits::default())
                .unwrap()
                .length
                .scale_to_si,
            (Real::from(127) / Real::from(5)).unwrap()
        );
    }
    for (old, new) in [
        ("LENGTH_MEASURE(25.4)", "LENGTH_MEASURE(0.)"),
        ("LENGTH_MEASURE(25.4)", "POSITIVE_LENGTH_MEASURE(-1.)"),
        ("LENGTH_MEASURE(25.4)", "PLANE_ANGLE_MEASURE(25.4)"),
        (
            "#3=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));",
            "#3=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));",
        ),
        ("DIMENSIONAL_EXPONENTS(1.,0.", "DIMENSIONAL_EXPONENTS(0.,1."),
        ("SI_UNIT($,.METRE.)", "SI_UNIT($,.RADIAN.)"),
        (
            "LENGTH_UNIT()NAMED_UNIT(*)",
            "LENGTH_UNIT()LENGTH_UNIT()NAMED_UNIT(*)",
        ),
        (
            "GLOBAL_UNIT_ASSIGNED_CONTEXT((#100))",
            "GLOBAL_UNIT_ASSIGNED_CONTEXT((#100,#3))",
        ),
    ] {
        let changed = source.replace(old, new);
        assert_ne!(changed, source);
        assert!(
            matches!(
                units(&changed, &ImportLimits::default()).unwrap_err().kind,
                ErrorKind::Invalid(_)
            ),
            "{changed}"
        );
    }
    let missing = source.replace(
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#100))",
        "GLOBAL_UNIT_ASSIGNED_CONTEXT(())",
    );
    assert!(matches!(
        units(&missing, &ImportLimits::default()).unwrap_err().kind,
        ErrorKind::Expected(_)
    ));
    let unknown = source.replace("SI_UNIT($,.METRE.)", "SI_UNIT($,.MYSTERY.)");
    assert!(matches!(
        units(&unknown, &ImportLimits::default()).unwrap_err().kind,
        ErrorKind::Unsupported(_)
    ));
}

#[test]
fn si_prefixes_are_exact_and_angle_categories_stay_distinct() {
    let source = support::unit_chain(0, "1.");
    for (prefix, exponent) in [
        ("YOTTA", 24_i32),
        ("ZETTA", 21),
        ("EXA", 18),
        ("PETA", 15),
        ("TERA", 12),
        ("GIGA", 9),
        ("MEGA", 6),
        ("KILO", 3),
        ("HECTO", 2),
        ("DECA", 1),
        ("DECI", -1),
        ("CENTI", -2),
        ("MILLI", -3),
        ("MICRO", -6),
        ("NANO", -9),
        ("PICO", -12),
        ("FEMTO", -15),
        ("ATTO", -18),
        ("ZEPTO", -21),
        ("YOCTO", -24),
    ] {
        let source = source.replace(
            "SI_UNIT($,.METRE.)",
            &format!("SI_UNIT(.{prefix}.,.METRE.)"),
        );
        let mut expected = Real::one();
        for _ in 0..exponent.unsigned_abs() {
            expected *= Real::from(10);
        }
        if exponent < 0 {
            expected = expected.inverse().unwrap();
        }
        assert_eq!(
            units(&source, &ImportLimits::default())
                .unwrap()
                .length
                .scale_to_si,
            expected
        );
    }
    let source=source.replace("GLOBAL_UNIT_ASSIGNED_CONTEXT((#3))","GLOBAL_UNIT_ASSIGNED_CONTEXT((#3,#4,#5))").replace("ENDSEC;END-ISO-10303-21;","#4=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n#5=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());\nENDSEC;END-ISO-10303-21;");
    let units = units(&source, &ImportLimits::default()).unwrap();
    assert_eq!(units.plane_angle.unwrap().kind, UnitKind::PlaneAngle);
    assert_eq!(units.solid_angle.unwrap().kind, UnitKind::SolidAngle);
}
