use hyperbrep::{
    Curve3, CurveParameterLocation, Direction, Matrix4, OperationContext, Orientation, Point2,
    Point3, Real, SolidRef, Vector3, WorkBudget, builder,
};
use hyperbrep_io::{
    part21::{NumberLimits, Value, ValueKind},
    step::{
        FacetedBody, FacetedStep, ImportLimits, StepFile, StepReadOptions, StepWriteOptions,
        WriteErrorKind, WriteLengthUnit, write_faceted,
    },
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
fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
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
fn reference(value: Value<'_, '_>) -> u64 {
    let ValueKind::Reference(id) = value.kind() else {
        panic!("expected reference")
    };
    id
}
fn triple(file: &StepFile<'_>, id: u64) -> [Real; 3] {
    argument(file, id, 1)
        .children()
        .map(|value| {
            let ValueKind::Real(number) = value.kind() else {
                panic!("expected real")
            };
            number.to_real(&NumberLimits::default()).unwrap()
        })
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}

// Inspect the emitted graph directly against the native source. These checks
// do not depend on the geometric STEP importer or on a float volume oracle.
fn check_graph(output: &FacetedStep<'_>, scale: &Real) {
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    assert_eq!(file.syntax().unresolved_references().count(), 0);
    for body in output.bodies() {
        for vertex in body.vertices() {
            let [x, y, z] = triple(&file, vertex.entity).map(|value| value * scale);
            assert_eq!(
                &Point3::new(x, y, z),
                vertex.source.point().try_coordinates().unwrap()
            );
        }
        for face in body.faces() {
            let placement = reference(argument(&file, face.plane, 1));
            let normal = Vector3::new(triple(&file, reference(argument(&file, placement, 2))));
            let direction = Vector3::new(triple(&file, reference(argument(&file, placement, 3))));
            assert_eq!(normal.dot(&direction), Real::zero());
            let authored = face
                .source
                .surface()
                .normal_at(&Point2::new(Real::zero(), Real::zero()))
                .unwrap();
            let signed = normal.dot(&authored);
            assert!(match face.source.orientation() {
                Orientation::Forward => signed > Real::zero(),
                Orientation::Reversed => signed < Real::zero(),
            });
            assert_eq!(
                argument(&file, face.entity, 3).kind(),
                ValueKind::Enumeration("T")
            );
        }
        for edge_use in body.edge_uses() {
            let polygon = argument(&file, edge_use.polygon, 1)
                .children()
                .map(reference)
                .collect::<Vec<_>>();
            let edge = edge_use.source.edge();
            let expected = match edge_use.source.direction() {
                Direction::Forward => [edge.start(), edge.end()],
                Direction::Reversed => [edge.end(), edge.start()],
            };
            for (offset, vertex) in expected.into_iter().enumerate() {
                let id = polygon[(edge_use.segment + offset) % polygon.len()];
                assert_eq!(
                    body.vertices()
                        .iter()
                        .find(|v| v.entity == id)
                        .unwrap()
                        .source,
                    vertex
                );
            }
        }
    }
}
fn check_solid(solid: SolidRef<'_>) {
    let output = write_faceted(
        &[FacetedBody {
            solid,
            name: "authored",
        }],
        &StepWriteOptions::new("native.step", "2026-10-15T00:00:00"),
        &context(),
    )
    .result
    .unwrap()
    .unwrap();
    check_graph(&output, &Real::one());
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    let replay = file
        .import_representation(
            output.representation(),
            &ImportLimits::default(),
            &context(),
        )
        .result
        .unwrap()
        .unwrap();
    assert_eq!(
        replay.bodies()[0].solid().volume().unwrap(),
        solid.volume().unwrap()
    );
    assert_eq!(
        replay.bodies()[0].edge_uses().count(),
        output.bodies()[0].edge_uses().len()
    );
}

#[test]
fn native_edits_shear_reflection_and_large_translations_keep_exact_oriented_geometry() {
    let bodies = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = bodies
        .model()
        .edges()
        .next()
        .unwrap()
        .split(ratio(1, 8).into())
        .unwrap();
    check_solid(split.model().solids().next().unwrap());
    for entries in [
        [1, 1, 0, 3, 0, 1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1],
        [-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1],
    ] {
        let moved = split
            .model()
            .transformed(&Matrix4::from_row_major(entries.map(Real::from)))
            .unwrap();
        check_solid(moved.solids().next().unwrap());
    }
    let moved = bodies
        .transformed(&Matrix4::affine_translation([
            Real::from(9_007_199_254_740_993i64),
            ratio(1, 1_000_000),
            -Real::one(),
        ]))
        .unwrap();
    check_solid(moved.bodies().next().unwrap());
}

#[test]
fn exact_output_units_preserve_inch_grid_and_nearby_distinct_bodies() {
    let first = builder::cuboid(point(0, 0, 0), point(1, 1, 1)).unwrap();
    let gap = ratio(1, 1_000_000) * ratio(1, 1_000_000);
    let second = first
        .transformed(&Matrix4::affine_translation([
            Real::one() + &gap,
            Real::zero(),
            Real::zero(),
        ]))
        .unwrap();
    let options = StepWriteOptions::new("gap.step", "2026-10-15T00:00:00");
    let output = write_faceted(
        &[
            FacetedBody {
                solid: first.bodies().next().unwrap(),
                name: "a",
            },
            FacetedBody {
                solid: second.bodies().next().unwrap(),
                name: "b",
            },
        ],
        &options,
        &context(),
    )
    .result
    .unwrap()
    .unwrap();
    check_graph(&output, &Real::one());
    let inch = ratio(127, 5000);
    let inches = first
        .transformed(&Matrix4::affine_nonuniform_scale([
            inch.clone(),
            inch.clone(),
            inch.clone(),
        ]))
        .unwrap();
    let mut options = options;
    options.length_unit = WriteLengthUnit {
        name: "inch",
        metres_per_unit: inch.clone(),
    };
    let output = write_faceted(
        &[FacetedBody {
            solid: inches.bodies().next().unwrap(),
            name: "inch cube",
        }],
        &options,
        &context(),
    )
    .result
    .unwrap()
    .unwrap();
    check_graph(&output, &inch);
    // A one-metre coordinate is 5000/127 inches; choosing inches cannot license rounding.
    let error = write_faceted(
        &[FacetedBody {
            solid: first.bodies().next().unwrap(),
            name: "metre cube",
        }],
        &options,
        &context(),
    )
    .result
    .unwrap()
    .unwrap_err();
    assert!(matches!(error.kind, WriteErrorKind::NonTerminatingDecimal));
}

#[test]
fn unsupported_curves_cavities_and_selected_images_fail_with_source_sites() {
    let options = StepWriteOptions::new("unsupported.step", "2026-10-15T00:00:00");
    let sphere = builder::sphere(Real::one()).unwrap();
    let cylinder = builder::cylinder(Real::one(), Real::one()).unwrap();
    let cavity = builder::sphere_with_voids(
        Real::from(3),
        &[builder::SphereVoid {
            center: Point3::origin(),
            radius: Real::one(),
        }],
    )
    .unwrap();
    for (bodies, expected) in [(&sphere, 0), (&cylinder, 1), (&cavity, 2)] {
        let error = write_faceted(
            &[FacetedBody {
                solid: bodies.bodies().next().unwrap(),
                name: "unsupported",
            }],
            &options,
            &context(),
        )
        .result
        .unwrap()
        .unwrap_err();
        assert!(
            matches!(
                (expected, &error.kind),
                (0, WriteErrorKind::UnsupportedSurface(_))
                    | (1, WriteErrorKind::UnsupportedCurve(_))
                    | (2, WriteErrorKind::VoidShells)
            ),
            "{error:?}"
        );
        assert_eq!(error.body, Some(0));
        assert!(error.entity.is_some());
    }
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let curve = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(ratio(n, 5), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("root")
    };
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(parameter)
        .unwrap();
    let error = write_faceted(
        &[
            FacetedBody {
                solid: source.bodies().next().unwrap(),
                name: "supported first",
            },
            FacetedBody {
                solid: split.model().solids().next().unwrap(),
                name: "selected image",
            },
        ],
        &options,
        &context(),
    )
    .result
    .unwrap()
    .unwrap_err();
    assert!(matches!(error.kind, WriteErrorKind::UnmaterializedPoint));
    assert_eq!(error.body, Some(1));
}
