//! Independently authored U/V source maps and hostile toroidal STEP inputs.
use super::torus_exchange as torus;
use hyperbrep::*;
use hyperbrep_io::{part21::ValueKind, step::*};
use torus::analytic::{
    self,
    cylinder::{append, argument, read, reverse_edge},
    equal, ratio,
};

struct Fixture {
    text: String,
    representation: u64,
    surface: u64,
    latitude: u64,
    meridian: u64,
}

fn fixture() -> Fixture {
    let body = builder::torus(3.into(), 1.into()).unwrap();
    let options = StepWriteOptions::new("source-maps.step", "2026-10-16T00:00:00");
    let output = analytic::export(
        &[StepBody {
            solid: body.bodies().next().unwrap(),
            name: "maps",
        }],
        &options,
    );
    let written = &output.bodies()[0];
    let find = |end: Point3| {
        written
            .edges()
            .iter()
            .find(|edge| {
                edge.source.start().point().try_coordinates()
                    == Some(&Point3::new(4.into(), 0.into(), 0.into()))
                    && edge.source.end().point().try_coordinates() == Some(&end)
            })
            .unwrap()
    };
    let latitude = find(Point3::new(0.into(), 4.into(), 0.into()));
    let meridian = find(Point3::new(3.into(), 0.into(), 1.into()));
    let surface = written.faces()[0].surface;
    // U=(2+2t)*angle_unit on the outer equator; V=(-1+3t)*angle_unit
    // on the zero-longitude meridian. Neither equation comes from writer maps.
    let records = format!(
        "
#10000=SURFACE_CURVE('',#{},(#10001),.PCURVE_S1.);
#10001=PCURVE('',#{surface},#10002);
#10002=DEFINITIONAL_REPRESENTATION('',(#10003),#10100);
#10003=LINE('',#10004,#10005);
#10004=CARTESIAN_POINT('',(2.,0.));
#10005=VECTOR('',#10006,2.);
#10006=DIRECTION('',(1.,0.));
#10010=SURFACE_CURVE('',#{},(#10011),.PCURVE_S1.);
#10011=PCURVE('',#{surface},#10012);
#10012=DEFINITIONAL_REPRESENTATION('',(#10013),#10100);
#10013=LINE('',#10014,#10015);
#10014=CARTESIAN_POINT('',(0.,-1.));
#10015=VECTOR('',#10016,3.);
#10016=DIRECTION('',(0.,1.));
#10100=PARAMETRIC_REPRESENTATION_CONTEXT('UV','');
",
        latitude.geometry, meridian.geometry
    );
    let text = append(
        &argument(
            &argument(output.text(), latitude.entity, 3, "#10000"),
            meridian.entity,
            3,
            "#10010",
        ),
        &records,
    );
    Fixture {
        text,
        representation: output.representation(),
        surface,
        latitude: latitude.entity,
        meridian: meridian.entity,
    }
}

fn import(text: &str, representation: u64) -> Result<ImportedRepresentation, Error> {
    read(text)
        .import_representation(
            representation,
            &ImportLimits::default(),
            &analytic::context(),
        )
        .result
        .unwrap()
}

fn maps(body: &ImportedBody, length: &Real, angle: &Real) {
    for (source, latitude) in [(10001, true), (10011, false)] {
        let edge = body
            .explicit_edges()
            .find(|edge| edge.parameters.iter().any(|p| p.curve == source))
            .unwrap();
        let map = edge.parameters.iter().find(|p| p.curve == source).unwrap();
        for i in 0..=4 {
            let fraction = ratio(i, 4);
            let t = &map.start + (&map.end - &map.start) * &fraction;
            let theta = if latitude {
                Real::from(2) + Real::from(2) * t
            } else {
                -Real::one() + Real::from(3) * t
            } * angle;
            let point = if latitude {
                Point3::new(
                    Real::from(4) * theta.clone().cos() * length,
                    Real::from(4) * theta.sin() * length,
                    Real::zero(),
                )
            } else {
                Point3::new(
                    (Real::from(3) + theta.clone().cos()) * length,
                    Real::zero(),
                    theta.sin() * length,
                )
            };
            let domain = edge.edge.curve().domain().clone();
            let observed = edge
                .edge
                .curve()
                .point_at(
                    &(domain.represented_start().unwrap()
                        + (domain.represented_end().unwrap()
                            - domain.represented_start().unwrap())
                            * fraction)
                        .into(),
                )
                .unwrap();
            assert!(observed.coincides_with(&point.into()).unwrap());
        }
    }
    torus::verify(
        body.model(),
        &(Real::from(6) * Real::pi() * Real::pi() * length * length * length),
        &(Real::from(12) * Real::pi() * Real::pi() * length * length),
        &[],
    );
    torus::seam::reload(body.model());
}

pub fn source_maps() {
    let fixture = fixture();
    for (length_prefix, length) in [("$", Real::one()), (".MILLI.", ratio(1, 1000))] {
        for (angle_prefix, angle) in [("$", Real::one()), (".MILLI.", ratio(1, 1000))] {
            let text = fixture
                .text
                .replace(
                    "SI_UNIT($,.METRE.)",
                    &format!("SI_UNIT({length_prefix},.METRE.)"),
                )
                .replace(
                    "SI_UNIT($,.RADIAN.)",
                    &format!("SI_UNIT({angle_prefix},.RADIAN.)"),
                );
            for master in [".CURVE_3D.", ".PCURVE_S1."] {
                let text = argument(&argument(&text, 10000, 3, master), 10010, 3, master);
                for text in [
                    text.clone(),
                    reverse_edge(&reverse_edge(&text, fixture.latitude), fixture.meridian),
                ] {
                    let result = import(&text, fixture.representation).unwrap();
                    maps(&result.bodies()[0], &length, &angle);
                    for source in [10000, 10001, 10003, 10010, 10011, 10013, 10100] {
                        assert!(result.interpreted_entities().contains(&source));
                    }
                    assert_eq!(
                        result.interpreted_entities().len() + result.retained_entities().len(),
                        read(&text).syntax().entities().len()
                    );
                }
            }
        }
    }
}

pub fn failures_and_recovery() {
    let fixture = fixture();
    for (major, minor) in [("0.", "1."), ("-1.", "1."), ("3.", "0."), ("3.", "-1.")] {
        let text = argument(
            &argument(&fixture.text, fixture.surface, 2, major),
            fixture.surface,
            3,
            minor,
        );
        let error = import(&text, fixture.representation).unwrap_err();
        assert!(
            matches!(
                error.kind,
                ErrorKind::Geometry(GeometryError::InvalidTorusRadii)
            ),
            "{error:?}"
        );
        assert_eq!(error.entity, Some(fixture.surface));
    }
    for (major, minor) in [("1.", "1."), ("0.5", "1.")] {
        let text = argument(
            &argument(&fixture.text, fixture.surface, 2, major),
            fixture.surface,
            3,
            minor,
        );
        let error = import(&text, fixture.representation).unwrap_err();
        assert!(
            matches!(
                error.kind,
                ErrorKind::Unsupported("non-ring toroidal surface")
            ),
            "{error:?}"
        );
        assert_eq!(error.entity, Some(fixture.surface));
    }
    // Wrong constant U and V, including a nearby decimal period, must fail the
    // complete spatial image check. Neither a seam declaration nor ambient
    // approximate policy can turn that decimal into tau.
    for (point, value) in [
        (10004, "(2.,0.000000000001)"),
        (10014, "(0.000000000001,-1.)"),
        (10004, "(2.,6.283185307179586)"),
        (10014, "(6.283185307179586,-1.)"),
    ] {
        let text = argument(&fixture.text, point, 1, value);
        for ambient in [false, true] {
            let failure = if ambient {
                hypercurve::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, || {
                    import(&text, fixture.representation)
                })
                .certified()
                .unwrap()
                .unwrap_err()
            } else {
                import(&text, fixture.representation).unwrap_err()
            };
            assert!(
                matches!(
                    failure.kind,
                    ErrorKind::Build(BuildError::EdgeUseSupportMismatch)
                ),
                "{failure:?}"
            );
        }
    }
    for (wrapper, pcurve, representation, point, value) in [
        (10000, 10001, 10002, 10004, "(2.,6.283185307179586)"),
        (10010, 10011, 10012, 10014, "(6.283185307179586,-1.)"),
    ] {
        let text = argument(
            &argument(&fixture.text, wrapper, 2, &format!("(#{pcurve},#10200)")),
            point,
            1,
            value,
        );
        let text = text.replace(
            &format!("#{wrapper}=SURFACE_CURVE"),
            &format!("#{wrapper}=SEAM_CURVE"),
        );
        let text = append(
            &text,
            &format!(
                "#10200=PCURVE('',#{},#{representation});\n",
                fixture.surface
            ),
        );
        assert!(matches!(
            import(&text, fixture.representation).unwrap_err().kind,
            ErrorKind::Build(BuildError::EdgeUseSupportMismatch)
        ));
    }
    let file = read(&fixture.text);
    let unit_list = file
        .syntax()
        .entities()
        .flat_map(|e| e.components())
        .find(|c| c.name() == "GLOBAL_UNIT_ASSIGNED_CONTEXT")
        .unwrap()
        .arguments()
        .next()
        .unwrap();
    let units = unit_list
        .children()
        .filter_map(|v| {
            let ValueKind::Reference(id) = v.kind() else {
                panic!("unit reference")
            };
            (!file
                .syntax()
                .entity(id)
                .unwrap()
                .components()
                .any(|c| c.name() == "PLANE_ANGLE_UNIT"))
            .then(|| format!("#{id}"))
        })
        .collect::<Vec<_>>()
        .join(",");
    let mut missing_angle = fixture.text.clone();
    let span = unit_list.span();
    missing_angle.replace_range(span.start..span.end, &format!("({units})"));
    assert!(matches!(
        import(&missing_angle, fixture.representation)
            .unwrap_err()
            .kind,
        ErrorKind::Expected("plane-angle unit for angular mapping")
    ));
    let malformed = argument(&fixture.text, 10006, 1, "(0.,1.)");
    assert!(import(&malformed, fixture.representation).is_err());
    maps(
        &import(&fixture.text, fixture.representation)
            .unwrap()
            .bodies()[0],
        &Real::one(),
        &Real::one(),
    );
}

pub fn admission() {
    let fixture = fixture();
    let file = read(&fixture.text);
    for limits in [
        ImportLimits {
            edge_uses: 0,
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
    ] {
        let error = file
            .import_representation(fixture.representation, &limits, &analytic::context())
            .result
            .unwrap()
            .unwrap_err();
        assert!(matches!(error.kind, ErrorKind::Limit { .. }));
    }
    let empty_budget = OperationContext::strict(WorkBudget {
        refinements: 0,
        promotions: 0,
    });
    let zero_solver_work = file
        .import_representation(
            fixture.representation,
            &ImportLimits::default(),
            &empty_budget,
        )
        .result
        .unwrap()
        .unwrap();
    // WorkBudget counts solver refinement/promotion, not all geometry work.
    // These analytic maps need neither; the independent import allowances above
    // must still fail before publishing a partial body.
    equal(
        &zero_solver_work.bodies()[0].solid().volume().unwrap(),
        &(Real::from(6) * Real::pi() * Real::pi()),
    );
    let recovered = file
        .import_representation(
            fixture.representation,
            &ImportLimits::default(),
            &analytic::context(),
        )
        .result
        .unwrap()
        .unwrap();
    equal(
        &recovered.bodies()[0].solid().volume().unwrap(),
        &(Real::from(6) * Real::pi() * Real::pi()),
    );
}
