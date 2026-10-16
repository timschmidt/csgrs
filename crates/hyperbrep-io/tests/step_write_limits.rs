use hyperbrep::{OperationContext, Point3, Real, WorkBudget, builder};
use hyperbrep_io::{
    part21::NumberLimits,
    step::{
        StepBody, StepFile, StepReadOptions, StepWriteOptions, WriteErrorKind, WriteResource,
        write_faceted,
    },
};
fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 0,
        promotions: 0,
    })
}
#[test]
fn output_counts_are_inclusive_and_rejections_can_be_retried() {
    let bodies =
        builder::cuboid(Point3::origin(), Point3::new(1.into(), 1.into(), 1.into())).unwrap();
    let inputs = [StepBody {
        solid: bodies.bodies().next().unwrap(),
        name: "unit",
    }];
    let mut options = StepWriteOptions::new("limits.step", "2026-10-15T00:00:00");
    let output = write_faceted(&inputs, &options, &context())
        .result
        .unwrap()
        .unwrap();
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    options.limits.output_bytes = output.text().len();
    options.limits.entities = file.syntax().entities().len();
    options.limits.edge_uses = 24;
    for resource in [
        WriteResource::OutputBytes,
        WriteResource::Entities,
        WriteResource::EdgeUses,
    ] {
        let mut small = options.clone();
        let limit = match resource {
            WriteResource::OutputBytes => &mut small.limits.output_bytes,
            WriteResource::Entities => &mut small.limits.entities,
            WriteResource::EdgeUses => &mut small.limits.edge_uses,
            _ => unreachable!(),
        };
        *limit -= 1;
        let error = write_faceted(&inputs, &small, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(
            matches!(error.kind,WriteErrorKind::Limit {resource: actual, ..} if actual == resource)
        );
        assert_eq!(
            write_faceted(&inputs, &options, &context())
                .result
                .unwrap()
                .unwrap()
                .text(),
            output.text()
        );
    }
    assert!(matches!(
        write_faceted(&[], &options, &context())
            .result
            .unwrap()
            .unwrap_err()
            .kind,
        WriteErrorKind::Invalid(_)
    ));
}

#[test]
fn decimal_significand_exponent_and_integer_storage_have_separate_limits() {
    for (coordinate, digits, exponent, expected) in [
        ("0.125", 3, 3, None),
        ("0.125", 2, 3, Some(WriteResource::DecimalDigits)),
        ("0.125", 3, 2, Some(WriteResource::DecimalExponent)),
        ("1000", 1, 3, None),
        ("1000", 1, 2, Some(WriteResource::DecimalExponent)),
        ("0.00001", 1, 5, None),
        ("0.00001", 1, 4, Some(WriteResource::DecimalExponent)),
        ("1", 0, 5, Some(WriteResource::DecimalDigits)),
    ] {
        let value = Real::new(coordinate.parse::<hyperreal::Rational>().unwrap());
        let bodies =
            builder::cuboid(Point3::origin(), Point3::new(value, 1.into(), 1.into())).unwrap();
        let mut options = StepWriteOptions::new("numbers.step", "2026-10-15T00:00:00");
        options.limits.number = NumberLimits { digits, exponent };
        let output = write_faceted(
            &[StepBody {
                solid: bodies.bodies().next().unwrap(),
                name: "number",
            }],
            &options,
            &context(),
        )
        .result
        .unwrap();
        if let Some(resource) = expected {
            assert!(
                matches!(output.unwrap_err().kind,WriteErrorKind::Limit {resource: actual, ..} if actual == resource),
                "{coordinate}"
            );
        } else {
            output.unwrap();
        }
    }
    // The source width fits 8 bits, but its fractional decimal coefficient does not.
    let eighth = (Real::one() / Real::from(128)).unwrap();
    let bodies =
        builder::cuboid(Point3::origin(), Point3::new(eighth, 1.into(), 1.into())).unwrap();
    let mut options = StepWriteOptions::new("bits.step", "2026-10-15T00:00:00");
    options.limits.number_bits = 8;
    assert!(matches!(
        write_faceted(
            &[StepBody {
                solid: bodies.bodies().next().unwrap(),
                name: "binary"
            }],
            &options,
            &context()
        )
        .result
        .unwrap()
        .unwrap_err()
        .kind,
        WriteErrorKind::Limit {
            resource: WriteResource::NumberBits,
            ..
        }
    ));
}

#[test]
fn escaped_string_expansion_obeys_the_physical_step_limit() {
    let bodies =
        builder::cuboid(Point3::origin(), Point3::new(1.into(), 1.into(), 1.into())).unwrap();
    let inputs = [StepBody {
        solid: bodies.bodies().next().unwrap(),
        name: "unit",
    }];
    for name in ["x".repeat(32_767), "'".repeat(16_383), "🙂".repeat(2047)] {
        let options = StepWriteOptions::new(&name, "2026-10-15T00:00:00");
        let output = write_faceted(&inputs, &options, &context())
            .result
            .unwrap()
            .unwrap();
        let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
        assert_eq!(file.header().name, name);
    }
    for name in ["x".repeat(32_768), "'".repeat(16_384), "🙂".repeat(2048)] {
        let options = StepWriteOptions::new(&name, "2026-10-15T00:00:00");
        assert!(matches!(
            write_faceted(&inputs, &options, &context())
                .result
                .unwrap()
                .unwrap_err()
                .kind,
            WriteErrorKind::Limit {
                resource: WriteResource::StringBytes,
                ..
            }
        ));
    }
}
