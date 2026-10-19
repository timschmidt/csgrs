use hyperlimit::{compare_reals, point3_equal};

use super::*;
use crate::Point2;
use crate::construction::{self as builder, ExtrusionVoid};

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

#[test]
fn cuboid_json_is_deterministic_and_revalidates_exactly() {
    let (model, solid) = builder::cuboid(p(-2, -3, -5), p(7, 11, 13)).unwrap();
    let json = model.to_json().unwrap();
    assert_eq!(json, model.to_json().unwrap());
    let rebuilt = ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(rebuilt.counts(), model.counts());
    assert_eq!(
        compare_reals(
            &rebuilt.solid_volume(solid).unwrap(),
            &model.solid_volume(solid).unwrap(),
            crate::STRICT_PREDICATES
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(rebuilt.to_json().unwrap(), json);
}

#[test]
fn rational_pcurve_persistence_retains_infinite_homogeneous_controls() {
    let controls = vec![
        hypercurve::HomogeneousControl2::new(Real::one(), Real::zero(), Real::one()),
        hypercurve::HomogeneousControl2::new(Real::zero(), Real::one(), Real::zero()),
        hypercurve::HomogeneousControl2::new(-Real::one(), Real::zero(), Real::one()),
    ];
    let hypercurve::Classification::Decided(curve) =
        crate::error::classified(RationalBezier2::from_homogeneous_controls(controls)).unwrap()
    else {
        panic!("the semicircle endpoints must be finite");
    };
    let mut builder = ArenaModelBuilder::new();
    let id = builder
        .pcurve(Pcurve::new(Curve2::from(curve.clone())))
        .unwrap();
    let model = builder.finish().unwrap();
    let json = model.to_json().unwrap();
    let rebuilt = ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let Some(CurveGeometry2::RationalBezier(restored)) =
        rebuilt.pcurve(id).unwrap().curve().geometry()
    else {
        panic!("persistence must retain the rational carrier");
    };
    assert_eq!(
        restored.homogeneous_controls(),
        curve.homogeneous_controls()
    );
    assert!(restored.affine_control_points().is_none());
    assert_eq!(rebuilt.to_json().unwrap(), json);
}

#[test]
fn homogeneous_nurbs_pcurve_persistence_retains_controls_and_periodicity() {
    use hypercurve::{HomogeneousControl2, NurbsCurve2, SplinePeriodicity2};
    let r = Real::from;
    let finite = NurbsCurve2::from_homogeneous_controls(
        2,
        vec![
            HomogeneousControl2::new(Real::one(), Real::zero(), Real::one()),
            HomogeneousControl2::new(Real::zero(), Real::one(), Real::zero()),
            HomogeneousControl2::new(-Real::one(), Real::zero(), Real::one()),
        ],
        vec![
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::one(),
            Real::one(),
        ],
        SplinePeriodicity2::NonPeriodic,
    )
    .unwrap();
    let periodic = NurbsCurve2::try_new_periodic(
        2,
        vec![
            curve_point2([r(0), r(0)]),
            curve_point2([r(4), r(0)]),
            curve_point2([r(4), r(4)]),
            curve_point2([r(0), r(4)]),
        ],
        vec![r(1), r(2), r(3), r(4)],
        (0..=4).map(r).collect(),
    )
    .unwrap();
    assert!(finite.affine_control_points().is_none());
    for source in [finite, periodic] {
        let mut builder = ArenaModelBuilder::new();
        let id = builder
            .pcurve(Pcurve::new(Curve2::from(source.clone())))
            .unwrap();
        let model = builder.finish().unwrap();
        let json = model.to_json().unwrap();
        let rebuilt = ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        let Some(CurveGeometry2::Nurbs(restored)) = rebuilt.pcurve(id).unwrap().curve().geometry()
        else {
            panic!("persistence must retain the NURBS carrier");
        };
        assert_eq!(
            restored.homogeneous_controls(),
            source.homogeneous_controls()
        );
        assert_eq!(restored.knots(), source.knots());
        assert_eq!(restored.periodicity(), source.periodicity());
        assert_eq!(restored.parameter_domain(), source.parameter_domain());
        if source.periodicity().is_periodic() {
            assert_eq!(
                restored.point_at_wrapped(&r(5)).unwrap(),
                source.point_at(&r(1)).unwrap()
            );
        }
        assert_eq!(rebuilt.to_json().unwrap(), json);
    }
}

#[test]
fn raw_import_rejects_invalid_references_during_validation() {
    let (model, _) = builder::cuboid(p(0, 0, 0), p(1, 1, 1)).unwrap();
    let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    raw["edges"][0]["start"] = serde_json::Value::from(999_u64);
    let json = serde_json::to_string(&raw).unwrap();
    assert!(matches!(
        ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::InvalidReference { .. }))
    ));
}

#[test]
fn raw_import_handles_deep_untrusted_json_on_a_growing_stack() {
    let mut json = String::from(
        r#"{"version":12,"selection":{"parameters":[],"point_sources":[],"pcurve_sources":[]},"vertices":[],"curves":[],"pcurves":[],"surfaces":[],"edges":[],"edge_uses":[],"wires":[],"faces":[],"shells":[],"solids":[],"ignored":"#,
    );
    for _ in 0..512 {
        json.push('[');
    }
    json.push('0');
    for _ in 0..512 {
        json.push(']');
    }
    json.push('}');
    ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
        .expect("deep untrusted JSON must use the growing stack adapter");
}

#[test]
fn void_shells_round_trip_through_untrusted_exact_persistence() {
    let outer = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::from(5), Real::zero()),
        Point2::new(Real::from(5), Real::from(5)),
        Point2::new(Real::zero(), Real::from(5)),
    ];
    let cavity = ExtrusionVoid {
        profile: vec![
            Point2::new(Real::one(), Real::one()),
            Point2::new(Real::from(4), Real::one()),
            Point2::new(Real::from(4), Real::from(4)),
            Point2::new(Real::one(), Real::from(4)),
        ],
        z_min: Real::one(),
        z_max: Real::from(4),
    };
    let (model, solid) =
        builder::extrude_with_voids(&outer, Real::zero(), Real::from(5), &[cavity]).unwrap();
    let json = model.to_json().unwrap();
    let rebuilt = ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(rebuilt.solid(solid).unwrap().voids().len(), 1);
    assert_eq!(
        compare_reals(
            &rebuilt.solid_volume(solid).unwrap(),
            &Real::from(98),
            crate::STRICT_PREDICATES
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(rebuilt.to_json().unwrap(), json);
}

#[test]
fn every_current_exact_curve_and_surface_carrier_round_trips() {
    let mut builder = ArenaModelBuilder::new();
    let line = Curve3::line(p(0, 0, 0), p(1, 2, 3)).unwrap();
    let rational = Curve3::rational_bezier(
        vec![p(0, 0, 0), p(1, 2, 0), p(3, 1, 0)],
        vec![Real::one(), Real::from(2), Real::one()],
    )
    .unwrap();
    let nurbs = Curve3::nurbs(
        2,
        vec![p(0, 0, 0), p(1, 2, 0), p(3, 1, 0)],
        vec![Real::one(), Real::from(2), Real::one()],
        vec![
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::one(),
            Real::one(),
        ],
    )
    .unwrap();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let circle = Curve3::circle_arc(
        p(0, 0, 0),
        Vector3::x(),
        Vector3::y(),
        Real::from(2),
        Real::zero(),
        half_pi.clone(),
    )
    .unwrap()
    .reversed()
    .unwrap();
    let ellipse = Curve3::ellipse_arc(
        p(1, 2, 3),
        Vector3::x(),
        Vector3::y(),
        Real::from(3),
        Real::from(2),
        Real::zero(),
        half_pi,
    )
    .unwrap();
    let curves = [line.clone(), rational, nurbs, circle, ellipse];
    for curve in curves {
        builder.curve(curve).unwrap();
    }

    let profile = Curve3::line(p(2, 0, -1), p(3, 0, 1)).unwrap();
    let control_points = vec![vec![p(0, 0, 0), p(1, 0, 0)], vec![p(0, 1, 1), p(1, 1, 1)]];
    let weights = vec![
        vec![Real::one(), Real::from(2)],
        vec![Real::from(3), Real::one()],
    ];
    let knots = vec![Real::zero(), Real::zero(), Real::one(), Real::one()];
    let quarter_pi = (Real::pi() / Real::from(4)).unwrap();
    let surfaces = [
        Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap(),
        Surface::cylinder(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Real::from(2),
        )
        .unwrap(),
        Surface::sphere(
            p(1, 2, 3),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Real::from(4),
        )
        .unwrap(),
        Surface::cone(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            quarter_pi,
        )
        .unwrap(),
        Surface::torus(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Real::from(5),
            Real::from(2),
        )
        .unwrap(),
        Surface::extrusion(profile.clone(), Vector3::z()).unwrap(),
        Surface::revolution(profile, p(0, 0, 0), Vector3::z()).unwrap(),
        Surface::rational_bezier(control_points.clone(), weights.clone()).unwrap(),
        Surface::nurbs(1, 1, control_points, weights, knots.clone(), knots).unwrap(),
    ];
    for surface in surfaces {
        builder.surface(surface).unwrap();
    }
    let arc = CircularArc2::try_from_center(
        CurvePoint2::new(Real::one(), Real::zero()),
        CurvePoint2::new(Real::zero(), Real::one()),
        CurvePoint2::new(Real::zero(), Real::zero()),
        false,
    )
    .unwrap();
    builder.pcurve(Pcurve::new(Curve2::from(arc))).unwrap();

    let model = builder.finish().unwrap();
    let json = model.to_json().unwrap();
    let rebuilt = ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(rebuilt.counts(), model.counts());
    assert_eq!(
        rebuilt
            .curves()
            .map(|(_, curve)| curve.kind())
            .collect::<Vec<_>>(),
        model
            .curves()
            .map(|(_, curve)| curve.kind())
            .collect::<Vec<_>>()
    );
    assert_eq!(
        rebuilt
            .surfaces()
            .map(|(_, surface)| surface.kind())
            .collect::<Vec<_>>(),
        model
            .surfaces()
            .map(|(_, surface)| surface.kind())
            .collect::<Vec<_>>()
    );
    for ((_, original), (_, decoded)) in model.curves().zip(rebuilt.curves()) {
        assert_eq!(
            point3_equal(
                &original
                    .evaluate_real(original.domain().represented_start().unwrap())
                    .unwrap(),
                &decoded
                    .evaluate_real(decoded.domain().represented_start().unwrap())
                    .unwrap(),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(true)
        );
    }
    assert_eq!(rebuilt.to_json().unwrap(), json);
}

#[test]
fn raw_import_revalidates_nonplanar_geometry_fields() {
    let mut builder = ArenaModelBuilder::new();
    builder
        .curve(
            Curve3::circle_arc(
                p(0, 0, 0),
                Vector3::x(),
                Vector3::y(),
                Real::one(),
                Real::zero(),
                Real::pi(),
            )
            .unwrap(),
        )
        .unwrap();
    let model = builder.finish().unwrap();
    let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    raw["curves"][0]["EllipseArc"]["direction"] = serde_json::Value::from(0);
    let json = serde_json::to_string(&raw).unwrap();
    assert!(matches!(
        ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::InvalidParameterDomain
        )))
    ));
}

#[test]
fn format_envelope_rejects_old_versions_before_decoding_their_schema() {
    assert!(matches!(
        ImportedModel::from_json(r#"{"version":8}"#, &crate::ArchiveLimits::default()),
        Err(PersistenceError::UnsupportedVersion(8))
    ));
}
