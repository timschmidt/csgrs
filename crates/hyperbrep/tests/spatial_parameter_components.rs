use hyperbrep::{
    ArchiveLimits, Curve3, CurveParameterLocation, GeometricPoint3, Matrix4, ModelBuilder, Point3,
    RawModel, Real, Surface, Vector3,
};
use hypercurve::{Curve2, NurbsCurve2, Point2 as Uv};

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn fixture() -> (Curve3, GeometricPoint3) {
    let tensor = Surface::rational_bezier(
        vec![
            vec![Point3::origin(), Point3::new(1.into(), 0.into(), 0.into())],
            vec![
                Point3::new(0.into(), 1.into(), 0.into()),
                Point3::new(1.into(), 1.into(), 0.into()),
            ],
        ],
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Uv::from_values(0, 0),
                Uv::from_values(0, 0),
                Uv::from_values(1, 0),
                Uv::from_values(0, 0),
            ],
            vec![Real::one(); 4],
            vec![0.into(), 0.into(), 1.into(), 2.into(), 3.into(), 3.into()],
        )
        .unwrap(),
    );
    let curve = Curve3::tensor_image(tensor, source).unwrap();
    let selector = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(q(n, 5), 0.into(), 0.into()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(roots) =
        selector.parameters_of(Point3::origin()).unwrap()
    else {
        panic!("selected root")
    };
    let query = selector.point_at(&roots[0]).unwrap();
    assert!(query.try_coordinates().is_none());
    (curve, query)
}

#[test]
fn proper_mixed_spatial_preimage_retains_its_continuum() {
    let (curve, query) = fixture();
    // The exact unit-chart abscissa is zero on [0,1/3], then 3t-1
    // on [1/3,2/3], then 3-3t on [2/3,1]. Other coordinates vanish.
    for t in [Real::zero(), q(1, 6), q(1, 3), Real::one()] {
        assert!(
            curve
                .point_at(&t.into())
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
    for t in [q(1, 2), q(2, 3), q(5, 6)] {
        assert!(
            !curve
                .point_at(&t.into())
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
    let location = curve
        .parameters_of(&query)
        .expect("complete exact [0,1/3] union {1}, retaining proper continuous components");
    let CurveParameterLocation::Components(components) = &location else {
        panic!("proper mixed inverse, not a finite set or the entire domain")
    };
    assert_eq!(components.len(), 2);
    let hypercurve::CurveParameterSet2::Range(range) = components[0].parameters().unwrap() else {
        panic!("retained interval")
    };
    assert_eq!(
        range.start().compare(&Real::zero().into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&q(1, 3).into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(components[0].endpoint_inclusion(), [true, true]);
    let hypercurve::CurveParameterSet2::Single(last) = components[1].parameters().unwrap() else {
        panic!("isolated final visit")
    };
    assert_eq!(
        last.compare(&Real::one().into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    let selected: hypercurve::CurveParameter2 =
        hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-1, 1, 0, 0, 0, 1].map(Real::from).to_vec(),
        )
        .unwrap()
        .isolate_unit_interval_roots()
        .unwrap()[0]
            .clone()
            .into();
    let selected: hyperbrep::Parameter = selected
        .affine_image(&q(1, 4), &Real::zero())
        .unwrap()
        .into();
    assert!(selected.try_real().is_none());
    let generated = curve.point_at_component(&components[0], &selected).unwrap();
    assert!(generated.coincides_with(&query).unwrap());
    assert!(
        curve
            .parameters_of(&generated)
            .unwrap()
            .contains(&selected, curve.domain())
            .unwrap()
    );
    for (t, expected) in [
        (q(-1, 2), false),
        (Real::zero(), true),
        (q(1, 6), true),
        (q(1, 3), true),
        (q(1, 2), false),
        (q(2, 3), false),
        (q(5, 6), false),
        (Real::one(), true),
        (2.into(), false),
    ] {
        assert_eq!(
            location.contains(&t.into(), curve.domain()).unwrap(),
            expected
        );
    }
}

fn assert_members(curve: &Curve3, query: &GeometricPoint3, probes: &[(Real, bool)]) {
    let locations = curve.parameters_of(query).unwrap();
    for (t, expected) in probes {
        assert_eq!(
            locations
                .contains(&t.clone().into(), curve.domain())
                .unwrap(),
            *expected
        );
        assert_eq!(
            curve
                .point_at(&t.clone().into())
                .unwrap()
                .coincides_with(query)
                .unwrap(),
            *expected
        );
    }
}

#[test]
fn mixed_inverse_reversal_restriction_and_affine_continuation() {
    let (curve, query) = fixture();
    let reversed = curve.reversed().unwrap();
    assert_members(
        &reversed,
        &query,
        &[
            (0.into(), true),
            (q(1, 3), false),
            (q(2, 3), true),
            (q(5, 6), true),
            (1.into(), true),
        ],
    );
    let trimmed = curve.subcurve(&q(1, 6), &q(5, 6)).unwrap();
    assert_members(
        &trimmed,
        &query,
        &[
            (0.into(), true),
            (q(1, 8), true),
            (q(1, 4), true),
            (q(1, 2), false),
            (1.into(), false),
        ],
    );
    let transform =
        Matrix4::from_row_major([1, 2, 0, 3, 0, -1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from));
    let moved = Curve3::affine_image(reversed, transform.clone()).unwrap();
    assert_members(
        &moved,
        &query.transformed(&transform).unwrap(),
        &[
            (0.into(), true),
            (q(1, 2), false),
            (q(2, 3), true),
            (1.into(), true),
        ],
    );
    let constant = curve.subcurve(&Real::zero(), &q(1, 3)).unwrap();
    let locations = constant.parameters_of(&query).unwrap();
    assert!(matches!(locations, CurveParameterLocation::EntireDomain));
    assert!(
        locations
            .contains(&q(1, 2).into(), constant.domain())
            .unwrap()
    );
    assert!(
        !locations
            .contains(&Real::from(2).into(), constant.domain())
            .unwrap()
    );
}

#[test]
fn planar_images_reuse_complete_source_components_and_reject_other_planes() {
    let (tensor, query) = fixture();
    let source = tensor.tensor_geometry().unwrap().source.clone();
    let planar =
        Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    assert_members(
        &planar,
        &query,
        &[
            (0.into(), true),
            (q(1, 6), true),
            (q(1, 3), true),
            (q(1, 2), false),
            (1.into(), true),
        ],
    );
    let wrong = GeometricPoint3::from(Point3::new(0.into(), 0.into(), 1.into()));
    assert!(matches!(
        planar.parameters_of(&wrong).unwrap(),
        CurveParameterLocation::None
    ));
    assert!(matches!(
        tensor.parameters_of(&wrong).unwrap(),
        CurveParameterLocation::None
    ));
}

#[test]
fn mixed_components_recompute_after_cold_native_geometry_replay() {
    if let Some(path) = std::env::var_os("HYPERBREP_COMPONENT_REPLAY") {
        let text = std::fs::read_to_string(path).unwrap();
        let replay = RawModel::from_json(&text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(text, replay.to_json().unwrap());
        check_replayed_geometry(&replay);
        return;
    }
    let (curve, query) = fixture();
    let mut builder = ModelBuilder::new();
    builder.curve(curve).unwrap();
    builder.vertex(query).unwrap();
    let model = builder.finish().unwrap();
    let text = model.to_json().unwrap();
    let replay = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(text, replay.to_json().unwrap());
    check_replayed_geometry(&replay);
    let path = std::env::temp_dir().join(format!(
        "csgrs-component-replay-{}-{}.json",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    struct Remove(std::path::PathBuf);
    impl Drop for Remove {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)
        .unwrap();
    let cleanup = Remove(path);
    use std::io::Write;
    file.write_all(text.as_bytes()).unwrap();
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "mixed_components_recompute_after_cold_native_geometry_replay",
            "--nocapture",
        ])
        .env("HYPERBREP_COMPONENT_REPLAY", &cleanup.0)
        .status()
        .unwrap();
    assert!(status.success(), "fresh-process exact component replay");
}

#[test]
fn coincident_spatial_endpoints_retain_distinct_one_sided_source_germs() {
    // S(u,v)=(u(u-1),v,0). The first source span is constant at u=0;
    // its successor jumps to u=1 at the same authored knot. Both world
    // points are zero, but their original UV germs remain distinct.
    let patch = Surface::rational_bezier(
        vec![
            vec![
                Point3::origin(),
                Point3::new(q(-1, 2), 0.into(), 0.into()),
                Point3::origin(),
            ],
            vec![
                Point3::new(0.into(), 1.into(), 0.into()),
                Point3::new(q(-1, 2), 1.into(), 0.into()),
                Point3::new(0.into(), 1.into(), 0.into()),
            ],
        ],
        vec![vec![Real::one(); 3]; 2],
    )
    .unwrap();
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Uv::from_values(0, 0),
                Uv::from_values(0, 0),
                Uv::from_values(1, 0),
                Uv::new(q(1, 2), 0.into()),
                Uv::from_values(1, 0),
            ],
            vec![Real::one(); 5],
            vec![
                0.into(),
                0.into(),
                1.into(),
                1.into(),
                2.into(),
                3.into(),
                3.into(),
            ],
        )
        .unwrap(),
    );
    let curve = Curve3::tensor_image(patch, source).unwrap();
    let (_, query) = fixture();
    let CurveParameterLocation::Components(components) = curve.parameters_of(&query).unwrap()
    else {
        panic!("germ-preserving mixed result")
    };
    assert_eq!(components.len(), 3);
    assert_eq!(
        components
            .iter()
            .map(|c| c.span_index())
            .collect::<Vec<_>>(),
        vec![0, 1, 2]
    );
    let hypercurve::CurveParameterSet2::Range(interval) = components[0].parameters().unwrap()
    else {
        panic!("first span")
    };
    let hypercurve::CurveParameterSet2::Single(jump) = components[1].parameters().unwrap() else {
        panic!("right knot germ")
    };
    assert_eq!(
        interval.end().compare(&jump).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        jump.compare(&q(1, 3).into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    let locations = curve.parameters_of(&query).unwrap();
    for (t, expected) in [
        (Real::zero(), true),
        (q(1, 6), true),
        (q(1, 3), true),
        (q(1, 2), false),
        (q(2, 3), false),
        (Real::one(), true),
    ] {
        assert_eq!(
            locations.contains(&t.into(), curve.domain()).unwrap(),
            expected
        );
    }
    // A scalar parameter cannot select either side of a discontinuous UV knot.
    // Keep that typed outcome and consume each explicit returned germ instead.
    assert!(
        matches!(curve.point_at(&q(1,3).into()),Err(hyperbrep::GeometryError::PlanarCurve(hypercurve::ExactCurveError::Blocked(cause))) if cause.reason()==hypercurve::UncertaintyReason::Boundary)
    );
    for component in &components {
        let points = match component.parameters().unwrap() {
            hypercurve::CurveParameterSet2::Single(p) => vec![p],
            hypercurve::CurveParameterSet2::Range(r) => vec![r.start().clone(), r.end().clone()],
        };
        for parameter in points {
            let actual = curve
                .point_at_component(component, &parameter.into())
                .unwrap();
            assert!(actual.coincides_with(&query).unwrap());
        }
    }
    let identity = Surface::rational_bezier(
        vec![
            vec![Point3::origin(), Point3::new(1.into(), 0.into(), 0.into())],
            vec![
                Point3::new(0.into(), 1.into(), 0.into()),
                Point3::new(1.into(), 1.into(), 0.into()),
            ],
        ],
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    let distinct =
        Curve3::tensor_image(identity, curve.tensor_geometry().unwrap().source.clone()).unwrap();
    let CurveParameterLocation::Components(left_germ) = distinct.parameters_of(&query).unwrap()
    else {
        panic!("left germ interval")
    };
    assert_eq!(left_germ.len(), 1);
    let endpoint = q(1, 3).into();
    assert!(distinct.point_at(&endpoint).is_err());
    let point = distinct
        .point_at_component(&left_germ[0], &endpoint)
        .unwrap();
    assert!(point.coincides_with(&query).unwrap());
    let invalid = left_germ[0]
        .affine_image_restricted(
            &Real::from(2),
            &Real::zero(),
            &hypercurve::CurveParameterRange2::unit(),
        )
        .unwrap()
        .unwrap();
    assert!(
        distinct
            .point_at_component(&invalid, &Real::zero().into())
            .is_err()
    );
}

fn check_replayed_geometry(replay: &hyperbrep::Model) {
    let curve = replay.curves().next().unwrap().geometry();
    let query = replay.vertices().next().unwrap().point();
    assert!(query.try_coordinates().is_none());
    assert_members(
        curve,
        query,
        &[
            (0.into(), true),
            (q(1, 6), true),
            (q(1, 3), true),
            (q(1, 2), false),
            (1.into(), true),
        ],
    );
    let next = curve
        .subcurve(&q(1, 6), &q(5, 6))
        .unwrap()
        .reversed()
        .unwrap();
    assert_members(
        &next,
        query,
        &[
            (0.into(), false),
            (q(1, 2), false),
            (q(3, 4), true),
            (1.into(), true),
        ],
    );
}
