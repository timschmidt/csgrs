//! Independently authored source maps, shared with native/WASM consumers.
use super::cylinder::{self, append, argument, equal, import, read, replay};
use hyperbrep::*;
use hyperbrep_io::step::{ErrorKind, ImportLimits, ImportedBody, Resource};

pub fn supplied(base: &str) -> String {
    let mut text = argument(base, 21, 3, "#200");
    text = argument(&text, 38, 3, "#230");
    text = argument(&text, 30, 3, "#260");
    // The axial edge lies exactly at u=0 in this independently authored frame.
    text = argument(&text, 49, 1, "(3.,4.,0.)");
    if read(base).syntax().entity(121).is_some() {
        text = argument(&argument(&text, 121, 3, "#200"), 138, 3, "#230");
    }
    append(
        &text,
        "
#200=SURFACE_CURVE('',#24,(#201,#211),.CURVE_3D.);
#201=PCURVE('',#45,#202);
#202=DEFINITIONAL_REPRESENTATION('',(#203),#290);
#203=LINE('',#204,#205);
#204=CARTESIAN_POINT('',(2.,7.));
#205=VECTOR('',#206,2.);
#206=DIRECTION('',(1.,0.));
#211=PCURVE('',#54,#212);
#212=DEFINITIONAL_REPRESENTATION('',(#213),#290);
#213=CIRCLE('',#214,5.);
#214=AXIS2_PLACEMENT_2D('',#215,#216);
#215=CARTESIAN_POINT('',(0.,0.));
#216=DIRECTION('',(0.,1.));
#230=SURFACE_CURVE('',#39,(#231,#241),.PCURVE_S1.);
#231=PCURVE('',#45,#232);
#232=DEFINITIONAL_REPRESENTATION('',(#233),#290);
#233=LINE('',#234,#235);
#234=CARTESIAN_POINT('',(-1.,0.));
#235=VECTOR('',#236,3.);
#236=DIRECTION('',(1.,0.));
#241=PCURVE('',#63,#242);
#242=DEFINITIONAL_REPRESENTATION('',(#243),#290);
#243=CIRCLE('',#244,5.);
#244=AXIS2_PLACEMENT_2D('',#245,#246);
#245=CARTESIAN_POINT('',(0.,0.));
#246=DIRECTION('',(3.,4.));
#260=SURFACE_CURVE('',#33,(#261),.PCURVE_S1.);
#261=PCURVE('',#45,#262);
#262=DEFINITIONAL_REPRESENTATION('',(#263),#290);
#263=LINE('',#264,#265);
#264=CARTESIAN_POINT('',(0.,-2.));
#265=VECTOR('',#266,4.);
#266=DIRECTION('',(0.,1.));
#290=PARAMETRIC_REPRESENTATION_CONTEXT('source UV','');
",
    )
}

fn point(body: &ImportedBody, source: u64, t: &Real, length: &Real, angle: &Real) -> Point3 {
    let circle = |theta: Real, z: i32| {
        Point3::new(
            Real::from(5) * theta.clone().cos() * length,
            Real::from(5) * theta.sin() * length,
            Real::from(z) * length,
        )
    };
    let cylinder = |u: Real, v: Real| {
        let u = u * angle;
        let c = u.clone().cos();
        let s = u.sin();
        Point3::new(
            (Real::from(3) * &c - Real::from(4) * &s) * length,
            (Real::from(4) * c + Real::from(3) * s) * length,
            v * length,
        )
    };
    match source {
        24 => circle(t * angle, 7),
        39 => circle(t * angle, 0),
        201 => cylinder(Real::from(2) + Real::from(2) * t, 7.into()),
        231 => cylinder(-Real::one() + Real::from(3) * t, Real::zero()),
        261 => cylinder(Real::zero(), -Real::from(2) + Real::from(4) * t),
        33 => Point3::new(
            Real::from(3) * length,
            Real::from(4) * length,
            Real::from(2) * t * length,
        ),
        211 => {
            let a = t * angle;
            Point3::new(
                -Real::from(5) * a.clone().sin() * length,
                Real::from(5) * a.cos() * length,
                Real::from(7) * length,
            )
        }
        241 => cylinder(t.clone(), Real::zero()),
        200 | 230 | 260 => {
            let master = body
                .explicit_edges()
                .find(|edge| edge.geometry == source)
                .unwrap()
                .master;
            point(body, master, t, length, angle)
        }
        _ => panic!("unexpected source {source}"),
    }
}

fn maps(body: &ImportedBody, length: &Real, angle: &Real) {
    for edge in body
        .explicit_edges()
        .filter(|edge| [200, 230, 260].contains(&edge.geometry))
    {
        assert_eq!(
            edge.parameters.len(),
            if edge.geometry == 260 { 3 } else { 4 }
        );
        let domain = edge.edge.curve().domain().clone();
        for fraction in 0..=4 {
            let s = (Real::from(fraction) / Real::from(4)).unwrap();
            let native = edge
                .edge
                .curve()
                .point_at(
                    &(domain.represented_start().unwrap()
                        + (domain.represented_end().unwrap()
                            - domain.represented_start().unwrap())
                            * &s)
                        .into(),
                )
                .unwrap();
            for map in edge.parameters {
                let source_parameter = &map.start + (&map.end - &map.start) * &s;
                let expected = point(body, map.curve, &source_parameter, length, angle);
                assert!(native.coincides_with(&expected.into()).unwrap_or_else(|error| panic!("source {} edge {} fraction {fraction}, length={length:?} angle={angle:?} parameter={source_parameter:?}: {error:?}", map.curve, edge.source)), "source {} fraction {fraction}", map.curve);
            }
        }
        assert_eq!(edge.edge.uses().count(), 2);
    }
}

pub fn source_maps() {
    assert_eq!(
        supplied(cylinder::CYLINDER),
        include_str!("../fixtures/supplied-cylinder.step")
    );
    for reverse in [false, true] {
        let base = if reverse {
            cylinder::reverse_edge(cylinder::CYLINDER, 21)
        } else {
            cylinder::CYLINDER.into()
        };
        for (master, chosen) in [
            (".CURVE_3D.", 24),
            (".PCURVE_S1.", 201),
            (".PCURVE_S2.", 211),
        ] {
            let text = argument(&supplied(&base), 200, 3, master);
            let imported = import(&text).unwrap();
            let body = &imported.bodies()[0];
            assert_eq!(
                body.explicit_edges()
                    .find(|edge| edge.source == 21)
                    .unwrap()
                    .master,
                chosen
            );
            maps(body, &Real::one(), &Real::one());
            cylinder::check(body.model());
            cylinder::check(&replay(body.model()));
        }
    }
}

pub fn representations() {
    let partial = supplied(&cylinder::partitioned());
    let mut reversed_bound = argument(
        &argument(&supplied(cylinder::CYLINDER), 19, 1, "(#44,#37,#29,#20)"),
        18,
        2,
        ".F.",
    );
    for id in [20, 29, 37, 44] {
        reversed_bound = argument(
            &reversed_bound,
            id,
            4,
            if [20, 29].contains(&id) { ".T." } else { ".F." },
        );
    }
    for text in [
        reversed_bound,
        partial.clone(),
        cylinder::reverse_edge(&partial, 21),
        cylinder::reverse_edge(&partial, 30),
        argument(&partial, 16, 1, "(#59,#50,#17,#117)"),
        argument(&supplied(cylinder::CYLINDER), 19, 1, "(#29,#37,#44,#20)"),
    ] {
        let imported = import(&text).unwrap();
        maps(&imported.bodies()[0], &Real::one(), &Real::one());
        cylinder::check(&replay(imported.bodies()[0].model()));
    }
    for (length_unit, length) in [
        ("$", Real::one()),
        (".MILLI.", (Real::one() / Real::from(1000)).unwrap()),
    ] {
        for (angle_unit, angle) in [
            ("$", Real::one()),
            (".MILLI.", (Real::one() / Real::from(1000)).unwrap()),
        ] {
            let text = supplied(cylinder::CYLINDER)
                .replace(
                    "SI_UNIT($,.METRE.)",
                    &format!("SI_UNIT({length_unit},.METRE.)"),
                )
                .replace(
                    "SI_UNIT($,.RADIAN.)",
                    &format!("SI_UNIT({angle_unit},.RADIAN.)"),
                );
            let imported = import(&text).unwrap();
            let body = &imported.bodies()[0];
            maps(body, &length, &angle);
            equal(
                &body.solid().volume().unwrap(),
                &(Real::from(175) * Real::pi() * &length * &length * &length),
            );
        }
    }
    let omitted = argument(&supplied(cylinder::CYLINDER), 214, 2, "$");
    let result = import(&omitted).unwrap();
    let edge = result.bodies()[0]
        .explicit_edges()
        .find(|edge| edge.source == 21)
        .unwrap();
    let pcurve = edge.parameters.iter().find(|map| map.curve == 211).unwrap();
    let spatial = edge.parameters.iter().find(|map| map.curve == 24).unwrap();
    equal(&pcurve.start, &spatial.start);
    equal(&pcurve.end, &spatial.end);
    cylinder::check(result.bodies()[0].model());
    let degree = append(
        &supplied(cylinder::CYLINDER).replace(
            "#70=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));",
            "#70=(CONVERSION_BASED_UNIT('degree',#300) NAMED_UNIT(#301) PLANE_ANGLE_UNIT());",
        ),
        "#300=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.02),#302);\n#301=DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\n#302=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n",
    );
    let result = import(&degree).unwrap();
    maps(
        &result.bodies()[0],
        &Real::one(),
        &(Real::one() / Real::from(50)).unwrap(),
    );
    cylinder::check(result.bodies()[0].model());
}

pub fn composition() {
    let text = supplied(cylinder::CYLINDER);
    let imported = import(&text).unwrap();
    let model = imported.bodies()[0].model();
    let original = model.to_json().unwrap();
    let edge = model
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::CircleArc)
        .unwrap();
    let split = edge
        .split((edge.curve().domain().represented_start().unwrap() + Real::pi()).into())
        .unwrap()
        .into_model();
    let reflected = split
        .transformed(&Matrix4::affine_nonuniform_scale([
            -Real::one(),
            Real::one(),
            Real::one(),
        ]))
        .unwrap();
    cylinder::check(&replay(&reflected));
    let a = BodySet::from_solid(model.solids().next().unwrap());
    let b = a
        .transformed(&Matrix4::affine_translation([
            20.into(),
            0.into(),
            0.into(),
        ]))
        .unwrap();
    let both = boolean::union(&a, &b).unwrap();
    assert_eq!(both.len(), 2);
    let remainder = boolean::difference(&both, &a).unwrap();
    let empty = boolean::difference(&remainder, &b).unwrap();
    assert!(empty.is_empty());
    let recovered = boolean::union(&empty, &a).unwrap();
    equal(
        &recovered.bodies().next().unwrap().volume().unwrap(),
        &(Real::from(175) * Real::pi()),
    );
    assert_eq!(model.to_json().unwrap(), original);
}

pub fn failures() {
    let good = supplied(cylinder::CYLINDER);
    for (index, text) in [
        argument(&good, 213, 2, "5.000000000001"),
        argument(&good, 204, 1, "(2.,7.000000000001)"),
        argument(&good, 206, 1, "(-1.,0.)"),
        argument(&good, 206, 1, "(1.,1.)"),
        argument(&good, 211, 1, "#63"),
        argument(&good, 200, 2, "()"),
        argument(&argument(&good, 200, 2, "(#201)"), 200, 3, ".PCURVE_S2."),
        argument(&good, 200, 2, "(#201,#211,#241)"),
        argument(&good, 200, 2, "(#45)"),
        argument(&good, 216, 1, "(0.,0.)"),
    ]
    .into_iter()
    .enumerate()
    {
        for _ in 0..2 {
            let error = import(&text).unwrap_err();
            let expected = match index {
                1 | 3 | 4 => matches!(
                    error.kind,
                    ErrorKind::Build(
                        BuildError::EdgeUseSupportMismatch
                            | BuildError::EdgeUseImageMismatch { .. }
                    )
                ),
                8 => matches!(error.kind, ErrorKind::Expected(_)),
                9 => matches!(error.kind, ErrorKind::Numeric(_)),
                _ => matches!(error.kind, ErrorKind::Invalid(_)),
            };
            assert!(expected, "case {index}: {error:?}");
        }
    }
    // Both pcurves on u=0 prove the axial image, but are not opposite seam bounds.
    let duplicate = append(
        &argument(&good, 260, 2, "(#261,#271)"),
        "#271=PCURVE('',#45,#262);\n",
    )
    .replace("#260=SURFACE_CURVE", "#260=SEAM_CURVE");
    assert!(matches!(
        import(&duplicate).unwrap_err().kind,
        ErrorKind::Invalid(_)
    ));
    // A nearby decimal is not an exact period. It cannot authorize repair.
    let decimal = append(
        &argument(&duplicate, 271, 2, "#272"),
        "#272=DEFINITIONAL_REPRESENTATION('',(#273),#290);\n#273=LINE('',#274,#265);\n#274=CARTESIAN_POINT('',(6.283185307179586,-2.));\n",
    );
    assert!(matches!(
        import(&decimal).unwrap_err().kind,
        ErrorKind::Build(BuildError::EdgeUseSupportMismatch)
    ));
    let ambient = hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, || {
        import(&decimal)
    });
    assert!(matches!(
        ambient.certified().unwrap().unwrap_err().kind,
        ErrorKind::Build(BuildError::EdgeUseSupportMismatch)
    ));
    let unsupported = good.replace("#213=CIRCLE", "#213=ELLIPSE");
    assert!(matches!(
        import(&unsupported).unwrap_err().kind,
        ErrorKind::Unsupported(_)
    ));
}

pub fn admission() {
    let text = supplied(cylinder::CYLINDER);
    let file = read(&text);
    let imported = import(&text).unwrap();
    assert_eq!(
        imported.interpreted_entities().len() + imported.retained_entities().len(),
        file.syntax().entities().len()
    );
    for id in [200, 201, 203, 211, 213, 230, 231, 241, 260, 261, 290] {
        assert!(imported.interpreted_entities().contains(&id));
    }
    let limits = ImportLimits {
        edge_uses: 0,
        ..ImportLimits::default()
    };
    let error = file
        .import_representation(10, &limits, &cylinder::context())
        .result
        .unwrap()
        .unwrap_err();
    assert!(matches!(
        error.kind,
        ErrorKind::Limit {
            resource: Resource::EdgeUses,
            ..
        }
    ));
    cylinder::check(import(&text).unwrap().bodies()[0].model());
}
