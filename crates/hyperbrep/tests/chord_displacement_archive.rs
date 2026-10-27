use hyperbrep::{
    ArchiveLimitExceeded, ArchiveLimits, ArchiveResource, ModelBuilder, Pcurve, PersistenceError,
    RawModel, Real,
};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCornerMode2, CurveFillet2, CurveParameter2, CurvePath2,
    CurvePointDefinition2, CurvePointLocations2, LineSeg2, Point2,
};
use serde_json::{Value, json};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn model(curve: Curve2) -> hyperbrep::Model {
    let mut builder = ModelBuilder::new();
    builder.pcurve(Pcurve::new(curve)).unwrap();
    builder.finish().unwrap()
}
fn replay(text: &str, limits: &ArchiveLimits) -> hyperbrep::Model {
    let model = RawModel::from_json(text, limits)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), text);
    model
}
#[test]
fn displaced_native_source_archive_counts_dependencies_before_scalar_replay() {
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(3, 4)).unwrap(),
    );
    assert!(source.geometry().is_some());
    for origin in [None, Some(Point2::from_values(2, 3).into())] {
        let point = CurvePointDefinition2::ChordDisplacement {
            source: source.clone(),
            origin: origin.clone(),
            at_end: false,
            distance: q(5, 2),
            translation: [q(7, 3), q(-1, 2)],
            left_normal: true,
        }
        .into_point()
        .unwrap();
        let curve = Curve2::try_line(point, Point2::from_values(10, 10).into()).unwrap();
        let text = model(curve).to_json().unwrap();
        // One output chord, two points, the source chord and its two points,
        // then one optional origin. No geometry coefficients are suppressed.
        let records = if origin.is_some() { 7 } else { 6 };
        let limits = ArchiveLimits {
            records,
            degree: 0,
            geometry_elements: 0,
            topology_references: 0,
            ..ArchiveLimits::default()
        };
        let smaller = ArchiveLimits {
            records: records - 1,
            ..limits
        };
        assert!(
            matches!(RawModel::from_json(&text,&smaller),Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded { resource:ArchiveResource::Records, observed, .. })) if observed==records)
        );
        let restored = replay(&text, &limits);
        let expected = if origin.is_some() {
            Point2::new(q(7, 3), 4.into())
        } else {
            Point2::new(q(1, 3), 1.into())
        };
        assert!(
            restored
                .pcurves()
                .next()
                .unwrap()
                .curve()
                .start()
                .coincides_with(&expected.clone().into())
                .unwrap()
        );
        let raw: Value = serde_json::from_str(&text).unwrap();
        // The native-line syntax is also a valid source. Its two coordinates
        // are inline, so they do not consume two planar-point records.
        let mut native = raw.clone();
        native["pcurves"][0]["Chord"]["start"]["ChordDisplacement"]["source"] = json!({
            "Line": {"start": [Real::from(0), Real::from(0)], "end": [Real::from(3), Real::from(4)]}
        });
        let native_limits = ArchiveLimits {
            records: records - 2,
            ..limits
        };
        assert!(matches!(
            RawModel::from_json(&native.to_string(), &ArchiveLimits { records: records - 3, ..limits }),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded { resource: ArchiveResource::Records, observed, .. })) if observed == records - 2
        ));
        let native_replay = RawModel::from_json(&native.to_string(), &native_limits)
            .unwrap()
            .validate()
            .unwrap();
        assert!(
            native_replay
                .pcurves()
                .next()
                .unwrap()
                .curve()
                .start()
                .coincides_with(&expected.into())
                .unwrap()
        );
        assert_eq!(native_replay.to_json().unwrap(), text);
        let mut bad = raw.clone();
        bad["pcurves"][0]["Chord"]["start"]["ChordDisplacement"]["source"]["Chord"]["start"]["Coordinates"] =
            json!(["invalid scalar", "invalid scalar"]);
        assert!(matches!(
            RawModel::from_json(&bad.to_string(), &smaller),
            Err(PersistenceError::ResourceLimit(_))
        ));
        assert!(RawModel::from_json(&bad.to_string(), &limits).is_err());
        let mut degenerate = raw.clone();
        let source =
            &mut degenerate["pcurves"][0]["Chord"]["start"]["ChordDisplacement"]["source"]["Chord"];
        source["end"] = source["start"].clone();
        assert!(
            RawModel::from_json(&degenerate.to_string(), &limits)
                .unwrap()
                .validate()
                .is_err()
        );
        let mut nonlinear = raw.clone();
        nonlinear["pcurves"][0]["Chord"]["start"]["ChordDisplacement"]["source"] = json!({"QuadraticBezier":[[Real::from(0),Real::from(0)],[Real::from(1),Real::from(2)],[Real::from(3),Real::from(4)]]});
        assert!(
            RawModel::from_json(&nonlinear.to_string(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .is_err()
        );
        replay(&text, &limits);
    }
}

#[test]
fn a_selected_fillet_cut_replays_through_the_public_brep_archive_before_split() {
    let mut coefficients = vec![Real::zero(); 8];
    coefficients[0] = (-1).into();
    coefficients[7] = 2.into();
    let beta: CurveParameter2 = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(q(3, 4), 1.into()).unwrap(),
        )
        .unwrap(),
    )
    .into();
    let at = |x, y| beta.affine_point(1.into(), x, 0.into(), y).unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::try_line(at(0.into(), 0.into()), at(1.into(), 0.into())).unwrap(),
        Curve2::try_line(at(1.into(), 0.into()), at(1.into(), 1.into())).unwrap(),
    ])
    .unwrap();
    let mut solutions = path
        .fillet_vertex(1, &CurveFillet2::new(q(1, 4)), CurveCornerMode2::TrimOnly)
        .unwrap()
        .into_solutions();
    assert_eq!(solutions.len(), 1);
    let filleted = solutions.pop().unwrap();
    let text = model(filleted.curves()[0].clone()).to_json().unwrap();
    let restored = replay(&text, &ArchiveLimits::default());
    let pcurve = restored.pcurves().next().unwrap();
    let curve = pcurve.curve();
    assert!(
        curve
            .start()
            .coincides_with(&at(0.into(), 0.into()))
            .unwrap()
    );
    assert!(curve.end().coincides_with(&at(q(3, 4), 0.into())).unwrap());
    let midpoint = at(q(3, 8), 0.into());
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&midpoint).unwrap()
    else {
        panic!("one selected midpoint")
    };
    assert_eq!(locations.len(), 1);
    let (first, second) = curve
        .split_at(locations[0].parameter().unwrap().clone())
        .unwrap();
    assert!(first.end().coincides_with(&midpoint).unwrap());
    assert!(second.start().coincides_with(&midpoint).unwrap());
    let again = model(second).to_json().unwrap();
    let replayed = replay(&again, &ArchiveLimits::default());
    assert!(
        replayed
            .pcurves()
            .next()
            .unwrap()
            .curve()
            .end()
            .coincides_with(&at(q(3, 4), 0.into()))
            .unwrap()
    );
}
