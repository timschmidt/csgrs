//! Independently authored STEP charts and exact failure controls.
use super::{analytic, equal, p, q, replay, round_trip};
use analytic::cylinder::{append, argument, read};
use hyperbrep::*;
use hyperbrep_io::step::*;

const LINE: &str = include_str!("../fixtures/cone-unit-line.step");

pub fn existing_surface_maps() {
    // The new public face map also covers existing faceted/elementary imports.
    // Check source formulas independently, including both angular torus axes.
    for kind in 0..4 {
        let body = match kind {
            0 | 1 => builder::cuboid(p(-2, -3, -4), p(2, 3, 4)).unwrap(),
            2 => builder::cylinder(5.into(), 7.into()).unwrap(),
            _ => builder::torus(3.into(), 1.into()).unwrap(),
        };
        for scale in [Real::one(), q(1, 1000)] {
            let mut options = StepWriteOptions::new("maps.step", "2026-10-17T00:00:00");
            options.length_unit = WriteLengthUnit {
                name: "source",
                metres_per_unit: scale,
            };
            let bodies = [StepBody {
                solid: body.bodies().next().unwrap(),
                name: "maps",
            }];
            let (text, representation) = if kind == 0 {
                let output = write_faceted(&bodies, &options, &analytic::context())
                    .result
                    .unwrap()
                    .unwrap();
                (output.text().to_owned(), output.representation())
            } else {
                let output = analytic::export(&bodies, &options);
                (output.text().to_owned(), output.representation())
            };
            for text in [
                text.clone(),
                text.replace("SI_UNIT($,.RADIAN.)", "SI_UNIT(.MILLI.,.RADIAN.)"),
            ] {
                let file = read(&text);
                let imported = file
                    .import_representation(
                        representation,
                        &ImportLimits::default(),
                        &analytic::context(),
                    )
                    .result
                    .unwrap()
                    .unwrap();
                analytic::source_maps(&file, &imported);
                equal(
                    &imported.bodies()[0].solid().volume().unwrap(),
                    &body.bodies().next().unwrap().volume().unwrap(),
                );
                replay(imported.bodies()[0].model());
            }
        }
    }
}

fn import(text: &str) -> Result<ImportedRepresentation, Error> {
    read(text)
        .import_representation(60, &ImportLimits::default(), &analytic::context())
        .result
        .unwrap()
}

pub fn reversed_generators_axes_units_and_source_maps() {
    for reverse_axis in [false, true] {
        for reverse_line in [false, true] {
            let mut text = argument(
                LINE,
                71,
                1,
                if reverse_axis {
                    "(0.,0.,1.)"
                } else {
                    "(0.,0.,-1.)"
                },
            );
            text = argument(
                &text,
                73,
                1,
                if reverse_line {
                    "(-1.,0.,2.)"
                } else {
                    "(1.,0.,-2.)"
                },
            );
            text = argument(
                &text,
                39,
                3,
                if reverse_axis ^ reverse_line {
                    ".F."
                } else {
                    ".T."
                },
            );
            text = argument(&text, 74, 2, "3.");
            // Rotate the loop and shell orders, including cap-first discovery.
            text = argument(&text, 37, 1, "(#28,#35,#36,#21)");
            text = argument(&text, 58, 1, "(#57,#39,#48)");
            for (length_prefix, length) in [(".MILLI.", q(1, 1000)), ("$", Real::one())] {
                for angle_prefix in ["$", ".MILLI."] {
                    let text = text
                        .replace(
                            "SI_UNIT(.MILLI.,.METRE.)",
                            &format!("SI_UNIT({length_prefix},.METRE.)"),
                        )
                        .replace(
                            "SI_UNIT($,.RADIAN.)",
                            &format!("SI_UNIT({angle_prefix},.RADIAN.)"),
                        );
                    let imported = import(&text).unwrap();
                    analytic::source_maps(&read(&text), &imported);
                    let model = replay(imported.bodies()[0].model());
                    let solid = model.solids().next().unwrap();
                    equal(
                        &solid.volume().unwrap(),
                        &(Real::from(78) * Real::pi() * &length * &length * &length),
                    );
                    assert_eq!(
                        solid
                            .classify_point(&Point3::new(
                                Real::zero(),
                                Real::zero(),
                                Real::from(3) * &length
                            ))
                            .unwrap(),
                        SolidPointLocation::Inside
                    );
                    let (round, _) = round_trip(
                        solid,
                        "reversed source",
                        &StepWriteOptions::new("reversed.step", "2026-10-17T00:00:00"),
                    );
                    equal(
                        &round.bodies().next().unwrap().volume().unwrap(),
                        &solid.volume().unwrap(),
                    );
                }
            }
        }
    }
}

pub fn supplied() -> String {
    // A 3-4-5 generator with authored speed 5 and a finite shifted origin.
    // Source V runs from 0 at radius 5 to 1 at radius 2. The axis is -Z;
    // U=0 is the (3/5,4/5) meridian. This reverses the native chart orientation.
    let mut text = LINE
        .replace("(1.2,1.6,6.)", "(1.2,1.6,4.)")
        .replace("(0.,0.,6.)", "(0.,0.,4.)");
    text = argument(&text, 24, 1, "(-9.,-12.,20.)");
    text = argument(&text, 70, 1, "(3.,4.,0.)");
    text = argument(&text, 72, 1, "#8");
    text = argument(&text, 73, 1, "(-9.,-12.,20.)");
    text = argument(&text, 74, 2, "5.");
    text = argument(&text, 39, 3, ".F.");
    text = argument(&text, 20, 3, "#200");
    text = argument(&text, 34, 3, "#210");
    text = argument(&text, 27, 3, "#220");
    append(
        &text,
        "
#200=SURFACE_CURVE('',#19,(#201),.PCURVE_S1.);
#201=PCURVE('',#12,#202);
#202=DEFINITIONAL_REPRESENTATION('',(#203),#290);
#203=LINE('',#204,#205);
#204=CARTESIAN_POINT('',(0.,1.));
#205=VECTOR('',#206,2.);
#206=DIRECTION('',(-1.,0.));
#210=SURFACE_CURVE('',#33,(#211),.CURVE_3D.);
#211=PCURVE('',#12,#212);
#212=DEFINITIONAL_REPRESENTATION('',(#213),#290);
#213=LINE('',#214,#215);
#214=CARTESIAN_POINT('',(0.,0.));
#215=VECTOR('',#206,3.);
#220=SURFACE_CURVE('',#26,(#221),.PCURVE_S1.);
#221=PCURVE('',#12,#222);
#222=DEFINITIONAL_REPRESENTATION('',(#223),#290);
#223=LINE('',#224,#225);
#224=CARTESIAN_POINT('',(0.,-2.));
#225=VECTOR('',#226,4.);
#226=DIRECTION('',(0.,1.));
#290=PARAMETRIC_REPRESENTATION_CONTEXT('cone source UV','');
",
    )
}

pub fn supplied_pcurve_parameters() {
    for angle_prefix in ["$", ".MILLI."] {
        let text = supplied().replace(
            "SI_UNIT($,.RADIAN.)",
            &format!("SI_UNIT({angle_prefix},.RADIAN.)"),
        );
        let imported = import(&text).unwrap();
        analytic::source_maps(&read(&text), &imported);
        let body = &imported.bodies()[0];
        equal(
            &body.solid().volume().unwrap(),
            &(Real::from(52) * Real::pi() * q(1, 1000) * q(1, 1000) * q(1, 1000)),
        );
        let angle = &imported.units().plane_angle.as_ref().unwrap().scale_to_si;
        for edge in body.explicit_edges() {
            for map in edge
                .parameters
                .iter()
                .filter(|m| [201, 211, 221].contains(&m.curve))
            {
                for i in 0..=4 {
                    let s = q(i, 4);
                    let t = &map.start + (&map.end - &map.start) * &s;
                    let (u, v) = match map.curve {
                        201 => (-Real::from(2) * t * angle, Real::one()),
                        211 => (-Real::from(3) * t * angle, Real::zero()),
                        221 => (Real::zero(), -Real::from(2) + Real::from(4) * t),
                        _ => unreachable!(),
                    };
                    let radius = Real::from(5) - Real::from(3) * &v;
                    let c = u.clone().cos();
                    let sin = u.sin();
                    let expected = Point3::new(
                        &radius * (q(3, 5) * &c + q(4, 5) * &sin) * q(1, 1000),
                        radius * (q(4, 5) * c - q(3, 5) * sin) * q(1, 1000),
                        Real::from(4) * v * q(1, 1000),
                    );
                    let (a, b) = edge.edge.domain().try_real_endpoints().unwrap();
                    let native = edge
                        .edge
                        .curve()
                        .point_at(&(a + (b - a) * s).into())
                        .unwrap();
                    assert!(native.coincides_with(&expected.into()).unwrap());
                }
            }
        }
        round_trip(
            body.solid(),
            "supplied",
            &StepWriteOptions::new("supplied.step", "2026-10-17T00:00:00"),
        );
    }
}

pub fn failures_and_limits() {
    let apex_only = argument(&argument(LINE, 13, 1, "(0.,0.,10.)"), 22, 1, "(0.,0.,10.)");
    assert!(matches!(
        import(&apex_only).unwrap_err().kind,
        ErrorKind::Geometry(GeometryError::SingularSurfaceParameter)
    ));
    assert!(matches!(
        import(&argument(LINE, 74, 2, "0.")).unwrap_err().kind,
        ErrorKind::Invalid(_)
    ));
    assert!(matches!(
        import(&argument(LINE, 71, 1, "(0.,0.,0.)"))
            .unwrap_err()
            .kind,
        ErrorKind::Numeric(_)
    ));
    assert!(matches!(
        import(include_str!("../fixtures/cone-finite-bspline.step"))
            .unwrap_err()
            .kind,
        ErrorKind::Unsupported("advanced spatial curve")
    ));
    for (direction, reason) in [
        ("(0.,0.,1.)", NonConicalGeneratorReason::Parallel),
        ("(1.,0.,0.)", NonConicalGeneratorReason::Perpendicular),
    ] {
        assert!(
            matches!(import(&argument(LINE,73,1,direction)).unwrap_err().kind,ErrorKind::Geometry(GeometryError::NonConicalGenerator(actual)) if actual==reason)
        );
    }
    // #70 is shared by the generator and axis; moving it translates the
    // support without making those two lines skew.
    let translated = argument(LINE, 70, 1, "(0.,0.000000000001,10.)");
    assert!(matches!(
        import(&translated).unwrap_err().kind,
        ErrorKind::Geometry(GeometryError::PointNotOnSurface)
    ));
    let skew = append(
        &argument(LINE, 75, 1, "#300"),
        "#300=CARTESIAN_POINT('',(0.,0.000000000001,10.));\n",
    );
    assert!(matches!(
        import(&skew).unwrap_err().kind,
        ErrorKind::Geometry(GeometryError::NonConicalGenerator(
            NonConicalGeneratorReason::Skew
        ))
    ));
    let missed = argument(LINE, 13, 1, "(1.200000000001,1.6,6.)");
    assert!(matches!(
        import(&missed).unwrap_err().kind,
        ErrorKind::Geometry(GeometryError::PointNotOnSurface)
    ));
    let supplied = supplied();
    for bad in [
        argument(&supplied, 204, 1, "(0.,1.000000000001)"),
        argument(&supplied, 224, 1, "(0.000000000001,-2.)"),
    ] {
        let error = import(&bad).unwrap_err();
        assert!(
            matches!(
                error.kind,
                ErrorKind::Build(
                    BuildError::EdgeUseSupportMismatch | BuildError::EdgeUseImageMismatch { .. }
                )
            ),
            "{error:?}"
        );
    }
    let duplicate = append(
        &argument(&supplied, 220, 2, "(#221,#231)"),
        "#231=PCURVE('',#12,#222);\n",
    )
    .replace("#220=SURFACE_CURVE", "#220=SEAM_CURVE");
    assert!(matches!(
        import(&duplicate).unwrap_err().kind,
        ErrorKind::Invalid(_)
    ));
    let file = read(LINE);
    for limits in [
        ImportLimits {
            entity_visits: 0,
            ..Default::default()
        },
        ImportLimits {
            scalars: 0,
            ..Default::default()
        },
        ImportLimits {
            edge_uses: 0,
            ..Default::default()
        },
    ] {
        assert!(matches!(
            file.import_representation(60, &limits, &analytic::context())
                .result
                .unwrap()
                .unwrap_err()
                .kind,
            ErrorKind::Limit { .. }
        ));
    }
    let imported = import(LINE).unwrap();
    let model = replay(imported.bodies()[0].model());
    let mut options = StepWriteOptions::new("limited.step", "2026-10-17T00:00:00");
    for limits in [
        WriteLimits {
            output_bytes: 0,
            ..Default::default()
        },
        WriteLimits {
            entities: 0,
            ..Default::default()
        },
        WriteLimits {
            number_bits: 1,
            ..Default::default()
        },
    ] {
        options.limits = limits;
        assert!(matches!(
            write_advanced(
                &[StepBody {
                    solid: model.solids().next().unwrap(),
                    name: "cone"
                }],
                &options,
                &analytic::context()
            )
            .result
            .unwrap()
            .unwrap_err()
            .kind,
            WriteErrorKind::Limit { .. }
        ));
    }
    let native = builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
    let empty = boolean::difference(&native, &native).unwrap();
    assert_eq!(empty.bodies().count(), 0);
    let shifted = native
        .transformed(&Matrix4::affine_translation([
            Real::pi(),
            Real::zero(),
            Real::zero(),
        ]))
        .unwrap();
    assert!(matches!(
        write_advanced(
            &[StepBody {
                solid: shifted.bodies().next().unwrap(),
                name: "symbolic"
            }],
            &StepWriteOptions::new("symbolic.step", "2026-10-17T00:00:00"),
            &analytic::context()
        )
        .result
        .unwrap()
        .unwrap_err()
        .kind,
        WriteErrorKind::NonRational
    ));
    assert_eq!(
        native
            .bodies()
            .next()
            .unwrap()
            .classify_point(&p(0, 0, 3))
            .unwrap(),
        SolidPointLocation::Inside
    );
    round_trip(
        native.bodies().next().unwrap(),
        "recovery",
        &StepWriteOptions::new("recovery.step", "2026-10-17T00:00:00"),
    );
}
