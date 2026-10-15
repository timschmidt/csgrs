use hyperbrep::{OperationContext, Point3, Real, WorkBudget, builder};
use hyperbrep_io::step::{
    FacetedBody, ImportLimits, StepFile, StepReadOptions, StepWriteOptions, WriteErrorKind,
    WriteLengthUnit, WriteLimits, WriteResource, write_faceted,
};

fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    })
}
fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn native_faceted_solids_export_every_body_hole_and_source_use_exactly() {
    for source in [
        include_str!("fixtures/faceted-box.step"),
        include_str!("fixtures/faceted-two-boxes.step"),
        include_str!("fixtures/faceted-through-hole.step"),
        include_str!("fixtures/faceted-tetrahedron.step"),
    ] {
        let file = StepFile::read(source.as_bytes(), &StepReadOptions::default()).unwrap();
        let imported = file
            .import_faceted_representation(502, &ImportLimits::default(), &context())
            .result
            .unwrap()
            .unwrap();
        let inputs = imported
            .bodies()
            .iter()
            .map(|body| FacetedBody {
                solid: body.solid(),
                name: body.name(),
            })
            .collect::<Vec<_>>();
        for millimetres in [false, true] {
            let mut options = StepWriteOptions::new("native.step", "2026-10-15T00:00:00");
            if millimetres {
                options.length_unit = WriteLengthUnit {
                    name: "millimetre",
                    metres_per_unit: ratio(1, 1000),
                };
            }
            let output = write_faceted(&inputs, &options, &context())
                .result
                .unwrap()
                .unwrap();
            assert_eq!(
                write_faceted(&inputs, &options, &context())
                    .result
                    .unwrap()
                    .unwrap()
                    .text(),
                output.text()
            );
            assert_eq!(output.bodies().len(), inputs.len());
            let file =
                StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
            assert_eq!(file.syntax().unresolved_references().count(), 0);
            let replay = file
                .import_faceted_representation(
                    output.representation(),
                    &ImportLimits::default(),
                    &context(),
                )
                .result
                .unwrap()
                .unwrap();
            assert_eq!(replay.bodies().len(), inputs.len());
            for ((written, original), replayed) in output
                .bodies()
                .iter()
                .zip(imported.bodies())
                .zip(replay.bodies())
            {
                assert_eq!(written.source(), original.solid());
                assert_eq!(written.vertices().len(), original.model().counts().vertices);
                assert_eq!(written.faces().len(), original.model().counts().faces);
                assert_eq!(written.wires().len(), original.model().counts().wires);
                assert_eq!(
                    written.edge_uses().len(),
                    original.model().counts().edge_uses
                );
                assert_eq!(
                    replayed.solid().volume().unwrap(),
                    original.solid().volume().unwrap()
                );
                assert_eq!(replayed.model().counts(), original.model().counts());
                for item in written.edge_uses() {
                    let wire = written
                        .wires()
                        .iter()
                        .find(|wire| wire.polygon == item.polygon)
                        .unwrap();
                    assert_eq!(item.source.wire(), wire.source);
                    assert_eq!(
                        wire.source.edge_uses().nth(item.segment).unwrap(),
                        item.source
                    );
                }
            }
        }
    }
}

#[test]
fn strict_writer_diagnoses_nonrepresentable_coordinates_units_and_limits() {
    let models = builder::cuboid(
        Point3::origin(),
        Point3::new(ratio(1, 3), Real::one(), Real::one()),
    )
    .unwrap();
    let inputs = [FacetedBody {
        solid: models.bodies().next().unwrap(),
        name: "third",
    }];
    let options = StepWriteOptions::new("third.step", "2026-10-15T00:00:00");
    let error = write_faceted(&inputs, &options, &context())
        .result
        .unwrap()
        .unwrap_err();
    assert!(matches!(error.kind, WriteErrorKind::NonTerminatingDecimal));
    assert_eq!(error.body, Some(0));
    assert!(error.entity.is_some());
    let models = builder::cuboid(
        Point3::origin(),
        Point3::new(Real::one(), Real::one(), Real::one()),
    )
    .unwrap();
    let inputs = [FacetedBody {
        solid: models.bodies().next().unwrap(),
        name: "unit",
    }];
    for value in [Real::zero(), -Real::one(), Real::pi(), ratio(1, 3)] {
        let mut changed = options.clone();
        changed.length_unit.metres_per_unit = value;
        let error = write_faceted(&inputs, &changed, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(matches!(
            error.kind,
            WriteErrorKind::Invalid(_)
                | WriteErrorKind::NonRational
                | WriteErrorKind::NonTerminatingDecimal
        ));
        assert_eq!(error.body, None);
    }
    for (limits, resource) in [
        (
            WriteLimits {
                output_bytes: 0,
                ..WriteLimits::default()
            },
            WriteResource::OutputBytes,
        ),
        (
            WriteLimits {
                entities: 0,
                ..WriteLimits::default()
            },
            WriteResource::Entities,
        ),
        (
            WriteLimits {
                edge_uses: 23,
                ..WriteLimits::default()
            },
            WriteResource::EdgeUses,
        ),
        (
            WriteLimits {
                number_bits: 0,
                ..WriteLimits::default()
            },
            WriteResource::NumberBits,
        ),
    ] {
        let mut changed = options.clone();
        changed.limits = limits;
        let error = write_faceted(&inputs, &changed, &context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(
            matches!(error.kind,WriteErrorKind::Limit{resource:actual,..} if actual==resource),
            "{error:?}"
        );
    }
}

#[test]
fn exact_file_metadata_and_repeated_occurrences_remain_explicit() {
    let models = builder::cuboid(
        Point3::origin(),
        Point3::new(ratio(1, 8), Real::one(), Real::one()),
    )
    .unwrap();
    let solid = models.bodies().next().unwrap();
    let input = FacetedBody {
        solid,
        name: "same 'body' \\ Ω🙂\n",
    };
    let options = StepWriteOptions::new("native'\\Ω🙂\n.step", "2026-10-15T00:00:00");
    let output = write_faceted(&[input, input], &options, &context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(output.bodies().len(), 2);
    assert_ne!(output.bodies()[0].entity(), output.bodies()[1].entity());
    assert_ne!(
        output.bodies()[0].vertices()[0].entity,
        output.bodies()[1].vertices()[0].entity
    );
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    assert_eq!(file.header().name, options.file_name);
    let replay = file
        .import_faceted_representation(
            output.representation(),
            &ImportLimits::default(),
            &context(),
        )
        .result
        .unwrap()
        .unwrap();
    assert_eq!(replay.bodies().len(), 2);
    assert!(
        replay
            .bodies()
            .iter()
            .all(|body| body.name() == input.name && body.solid().volume().unwrap() == ratio(1, 8))
    );
}
