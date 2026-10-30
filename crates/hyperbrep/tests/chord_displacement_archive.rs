#[path = "support/archive_graph.rs"]
mod archive_graph;
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
        // Shared coordinate/construction/chord nodes and each encoded reference
        // are records. An optional origin adds its node and reference.
        let records = if origin.is_some() { 14 } else { 12 };
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
        // Native-line syntax remains valid. Replacing the source reference
        // leaves its old graph definitions present; all are still admitted.
        let mut native = raw.clone();
        archive_graph::at_mut(&mut native, "/pcurves/0/Chord/start/ChordDisplacement")["source"] = json!({
            "Line": {"start": [Real::from(0), Real::from(0)], "end": [Real::from(3), Real::from(4)]}
        });
        let native_limits = ArchiveLimits { records, ..limits };
        assert!(matches!(
            RawModel::from_json(&native.to_string(), &ArchiveLimits { records: records - 1, ..limits }),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded { resource: ArchiveResource::Records, observed, .. })) if observed == records
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
        archive_graph::at_mut(
            &mut bad,
            "/pcurves/0/Chord/start/ChordDisplacement/source/Chord/start",
        )["Coordinates"] = json!(["invalid scalar", "invalid scalar"]);
        assert!(matches!(
            RawModel::from_json(&bad.to_string(), &smaller),
            Err(PersistenceError::ResourceLimit(_))
        ));
        assert!(RawModel::from_json(&bad.to_string(), &limits).is_err());
        let mut degenerate = raw.clone();
        let source = archive_graph::at_mut(
            &mut degenerate,
            "/pcurves/0/Chord/start/ChordDisplacement/source/Chord",
        );
        source[1] = source[0].clone();
        assert!(
            RawModel::from_json(&degenerate.to_string(), &limits)
                .unwrap()
                .validate()
                .is_err()
        );
        let mut nonlinear = raw.clone();
        archive_graph::at_mut(&mut nonlinear, "/pcurves/0/Chord/start/ChordDisplacement")["source"] = json!({"QuadraticBezier":[[Real::from(0),Real::from(0)],[Real::from(1),Real::from(2)],[Real::from(3),Real::from(4)]]});
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

#[test]
fn repeated_chord_displacements_archive_shared_dependencies_once() {
    let mut sizes = Vec::new();
    for depth in [3, 6] {
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
        let mut curve = Curve2::try_line(at(0.into(), 0.into()), at(1.into(), 0.into())).unwrap();
        for _ in 0..depth {
            let displaced = |at_end| {
                CurvePointDefinition2::ChordDisplacement {
                    source: curve.clone(),
                    origin: None,
                    at_end,
                    distance: 1.into(),
                    translation: [0.into(), 0.into()],
                    left_normal: true,
                }
                .into_point()
                .unwrap()
            };
            curve = Curve2::try_line(displaced(false), displaced(true)).unwrap();
        }
        let text = model(curve).to_json().unwrap();
        let cold = replay(&text, &ArchiveLimits::default());
        let pcurve = cold.pcurves().next().unwrap();
        let curve = pcurve.curve();
        assert!(
            curve
                .start()
                .coincides_with(&at(0.into(), Real::from(depth)))
                .unwrap()
        );
        assert!(
            curve
                .end()
                .coincides_with(&at(1.into(), Real::from(depth)))
                .unwrap()
        );
        let CurvePointLocations2::Locations(locations) = curve
            .point_locations(&at(q(1, 2), Real::from(depth)))
            .unwrap()
        else {
            panic!("one independent midpoint")
        };
        assert_eq!(locations.len(), 1);
        sizes.push(text.len());
    }
    eprintln!("chord dependency wire sizes at depths 3/6: {sizes:?}");
    assert!(
        sizes[1] <= 3 * sizes[0],
        "doubling the shared dependency depth must not expand an inline binary tree: {sizes:?}"
    );
}

#[test]
fn oblique_displacement_replays_before_independent_incidence_split_and_projection() {
    use hypercurve::{CurveCertainty, PredicatePolicy, Similarity2};
    let selected = || -> CurveParameter2 {
        let mut coefficients = vec![Real::zero(); 8];
        coefficients[0] = (-1).into();
        coefficients[7] = 2.into();
        BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
                BezierParameterInterval::try_new(q(3, 4), 1.into()).unwrap(),
            )
            .unwrap(),
        )
        .into()
    };
    let beta = selected();
    let at = |x, y| beta.affine_point(1.into(), x, 0.into(), y).unwrap();
    let source = Curve2::try_line(at(0.into(), 0.into()), at(3.into(), 4.into())).unwrap();
    let displaced = |at_end| {
        CurvePointDefinition2::ChordDisplacement {
            source: source.clone(),
            origin: None,
            at_end,
            distance: q(5, 2),
            translation: [q(7, 3), q(-1, 2)],
            left_normal: false,
        }
        .into_point()
        .unwrap()
    };
    let transform = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let curve = Curve2::try_line(displaced(false), displaced(true))
        .unwrap()
        .transform_similarity(&transform)
        .unwrap();
    let text = model(curve).to_json().unwrap();
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let restored = replay(&text, &ArchiveLimits::default());
            let pcurve = restored.pcurves().next().unwrap();
            let curve = pcurve.curve();
            // Derive the coefficients independently from S(beta+23/6, 3/2)
            // and displacement S(3,4)=(-7,24), using a separately selected root.
            let independent = selected();
            let expected = |x, y| independent.affine_point(3.into(), x, 4.into(), y).unwrap();
            assert!(
                curve
                    .start()
                    .coincides_with(&expected(q(15, 2), q(113, 6)))
                    .unwrap()
            );
            assert!(
                curve
                    .end()
                    .coincides_with(&expected(q(1, 2), q(257, 6)))
                    .unwrap()
            );
            let midpoint = expected(4.into(), q(185, 6));
            let CurvePointLocations2::Locations(locations) =
                curve.point_locations(&midpoint).unwrap()
            else {
                panic!("one independent incident point")
            };
            assert_eq!(locations.len(), 1);
            let (first, second) = curve
                .split_at(locations[0].parameter().unwrap().clone())
                .unwrap();
            assert!(first.end().coincides_with(&midpoint).unwrap());
            assert!(second.start().coincides_with(&midpoint).unwrap());
            let mut builder = ModelBuilder::new();
            for piece in [first, second] {
                builder.pcurve(Pcurve::new(piece)).unwrap();
            }
            let split_text = builder.finish().unwrap().to_json().unwrap();
            let again = replay(&split_text, &ArchiveLimits::default());
            let path =
                CurvePath2::try_new(again.pcurves().map(|p| p.curve().clone()).collect()).unwrap();
            assert_eq!(path.curves().len(), 2);
            assert!(path.curves()[0].end().coincides_with(&midpoint).unwrap());
            assert!(path.curves()[1].start().coincides_with(&midpoint).unwrap());
            let projection = path
                .project_to_finite_polyline(
                    &hypercurve::FiniteProjectionOptions::try_new(1e-6).unwrap(),
                )
                .unwrap();
            let b = 0.5_f64.powf(1.0 / 7.0);
            for (point, [x, y]) in [
                projection.points().first().unwrap(),
                projection.points().last().unwrap(),
            ]
            .into_iter()
            .zip([
                [3.0 * b + 7.5, 4.0 * b + 113.0 / 6.0],
                [3.0 * b + 0.5, 4.0 * b + 257.0 / 6.0],
            ]) {
                assert!((point[0] - x).abs() < 1e-10 && (point[1] - y).abs() < 1e-10);
            }
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    }
}
