#[path = "archive_graph.rs"]
mod archive_graph;
use hyperbrep::*;
use hypercurve::{Curve2, CurveError, CurvePointDefinition2, ExactCurveError, Point2 as Uv};
use serde_json::{Value, json};

fn model() -> Model {
    let p = |x, y| Uv::from_values(x, y).into();
    let point = CurvePointDefinition2::SupportingLineIntersection {
        first: [p(0, 0), p(1, 0)],
        second: [p(2, 1), p(2, 2)],
    }
    .into_point()
    .unwrap();
    let curve = Curve2::try_line(point, p(3, 0)).unwrap();
    assert!(curve.geometry().is_none());
    let mut builder = ModelBuilder::new();
    builder.pcurve(Pcurve::new(curve)).unwrap();
    builder.finish().unwrap()
}

fn replay(json: &str, limits: &ArchiveLimits) -> Model {
    let restored = RawModel::from_json(json, limits)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), json);
    let point = restored.pcurves().next().unwrap().curve().start();
    assert!(point.coincides_with(&Uv::from_values(2, 0).into()).unwrap());
    restored
}

pub fn supporting_line_archive_limits_and_forgery_recover() {
    let json = model().to_json().unwrap();
    let raw: Value = serde_json::from_str(&json).unwrap();
    assert!(
        archive_graph::at(&raw, "/pcurves/0/Chord/start/SupportingLineIntersection").is_object()
    );
    // Five coordinate nodes, one intersection node and four source references,
    // one chord node with two endpoint references, and its pcurve reference.
    let exact = ArchiveLimits {
        records: 14,
        degree: 0,
        geometry_elements: 0,
        topology_references: 0,
        ..ArchiveLimits::default()
    };
    assert!(matches!(
        RawModel::from_json(
            &json,
            &ArchiveLimits {
                records: 13,
                ..exact
            }
        ),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Records,
            limit: 13,
            observed: 14
        }))
    ));
    replay(&json, &exact);
    for (mode, cause) in [
        (0, CurveError::ZeroLengthLine),
        (1, CurveError::ParallelLineSupports),
        (2, CurveError::ParallelLineSupports),
    ] {
        let mut bad = raw.clone();
        let definition = archive_graph::at_mut(
            &mut bad,
            "/pcurves/0/Chord/start/SupportingLineIntersection",
        );
        let p = |x: i32, y: i32| json!({"Coordinates":[Real::from(x),Real::from(y)]});
        match mode {
            0 => definition["first"][1] = definition["first"][0].clone(),
            1 => definition["second"] = json!([p(0, 1), p(1, 1)]),
            _ => definition["second"] = json!([p(2, 0), p(3, 0)]),
        }
        let error = RawModel::from_json(&bad.to_string(), &exact)
            .and_then(RawModel::validate)
            .unwrap_err();
        assert!(
            matches!(error,PersistenceError::Build(BuildError::Geometry(GeometryError::PlanarCurve(ExactCurveError::Invalid{cause:actual,..}))) if actual==cause)
        );
        replay(&json, &exact);
    }
    let mut bad = raw.clone();
    archive_graph::at_mut(
        &mut bad,
        "/pcurves/0/Chord/start/SupportingLineIntersection/first/0",
    )["Coordinates"] = json!(["invalid scalar", "invalid scalar"]);
    // Metadata exhaustion wins before exact scalar construction.
    assert!(matches!(
        RawModel::from_json(
            &bad.to_string(),
            &ArchiveLimits {
                records: 2,
                ..exact
            }
        ),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Records,
            limit: 2,
            observed: 3
        }))
    ));
    assert!(RawModel::from_json(&bad.to_string(), &exact).is_err());
    replay(&json, &exact);
}

pub fn nested_supporting_line_archives_replay_with_record_limits() {
    let p = |x: i32, y: i32| -> hypercurve::CurvePoint2 { Uv::from_values(x, y).into() };
    for depth in [1, 2, 4, 8] {
        let mut point = CurvePointDefinition2::SupportingLineIntersection {
            first: [p(0, 0), p(1, 0)],
            second: [p(2, 1), p(2, 2)],
        }
        .into_point()
        .unwrap();
        for i in 1..depth {
            point = CurvePointDefinition2::SupportingLineIntersection {
                first: [point, p(i + 3, 0)],
                second: [p(i + 2, 1), p(i + 2, 2)],
            }
            .into_point()
            .unwrap();
        }
        let mut builder = ModelBuilder::new();
        builder
            .pcurve(Pcurve::new(
                Curve2::try_line(point, p(depth + 2, 0)).unwrap(),
            ))
            .unwrap();
        let json = builder.finish().unwrap().to_json().unwrap();
        // The last authored endpoint is already a source endpoint after depth one.
        let records = 8 * usize::try_from(depth).unwrap() + if depth == 1 { 6 } else { 5 };
        let exact = ArchiveLimits {
            records,
            ..ArchiveLimits::default()
        };
        assert!(
            matches!(RawModel::from_json(&json,&ArchiveLimits{records:records-1,..exact}),Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded{resource:ArchiveResource::Records,observed,..})) if observed==records)
        );
        let cold = RawModel::from_json(&json, &exact)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), json);
        assert!(
            cold.pcurves()
                .next()
                .unwrap()
                .curve()
                .start()
                .coincides_with(&p(depth + 1, 0))
                .unwrap()
        );
    }
}
