use hyperbrep::*;
use hypercurve::{
    BezierParameter2, Curve2, CurveParameter2, CurveParameterRange2, CurvePoint2,
    CurvePointDefinition2, Point2 as Uv, RationalBezier2, RationalMap2,
};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use serde_json::{Value, json};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn fiber(n: i32) -> CurvePoint2 {
    CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[0, 0, 0, 1], &[-1]]),
        parameter: BezierParameter2::Exact(n.into()).into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}
fn pair(sources: [CurvePoint2; 2]) -> CurvePoint2 {
    CurvePointDefinition2::RationalQueryPair {
        sources,
        numerators: [polynomial(&[&[0, 1]]), polynomial(&[&[0, 1]])],
        denominators: [polynomial(&[&[1]]), polynomial(&[&[1]])],
    }
    .into_point()
    .unwrap()
}
fn model() -> Model {
    let point = pair([fiber(2), fiber(3)]);
    let end = point
        .rational_image(&RationalMap2::new(
            [polynomial(&[&[1], &[1]]), polynomial(&[&[0, 1]])],
            polynomial(&[&[1]]),
        ))
        .unwrap();
    let parameter = point
        .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[4]]))
        .unwrap()
        .parameter()
        .unwrap();
    let source: Curve2 = RationalBezier2::try_new(
        vec![
            Uv::new(0.into(), 0.into()),
            Uv::new(q(1, 2), 0.into()),
            Uv::new(1.into(), 1.into()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap()
    .into();
    let restricted = source
        .restrict_source(
            CurveParameterRange2::try_new(CurveParameter2::from(Real::zero()), parameter).unwrap(),
        )
        .unwrap();
    let mut builder = ModelBuilder::new();
    builder
        .pcurve(Pcurve::new(Curve2::try_line(point, end).unwrap()))
        .unwrap();
    builder.pcurve(Pcurve::new(restricted)).unwrap();
    builder.finish().unwrap()
}
fn replay(text: &str) -> Model {
    RawModel::from_json(text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap()
}
fn verify(text: &str) {
    let model = replay(text);
    let curves = model
        .pcurves()
        .map(|p| p.curve().clone())
        .collect::<Vec<_>>();
    assert_eq!(curves.len(), 2);
    let point = curves[0].start();
    for relation in [
        polynomial(&[&[-2], &[], &[], &[1]]),
        polynomial(&[&[-3, 0, 0, 1]]),
    ] {
        assert_eq!(
            point
                .rational_form(relation, polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
    }
    let endpoint = curves[1].end();
    assert_eq!(
        endpoint
            .rational_form(polynomial(&[&[0, -1], &[], &[1]]), polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    let expected = point
        .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[4]]))
        .unwrap()
        .parameter()
        .unwrap();
    assert_eq!(
        curves[1]
            .parameter_domain()
            .end()
            .compare(&expected)
            .unwrap(),
        Ordering::Equal
    );
    let half = expected.affine_image(&q(1, 2), &Real::zero()).unwrap();
    let (first, second) = curves[1].split_at(half).unwrap();
    assert!(first.end().coincides_with(&second.start()).unwrap());
    let mut builder = ModelBuilder::new();
    for curve in [first, second] {
        builder.pcurve(Pcurve::new(curve)).unwrap();
    }
    let next = replay(&builder.finish().unwrap().to_json().unwrap());
    assert!(
        next.pcurves()
            .last()
            .unwrap()
            .curve()
            .end()
            .coincides_with(&endpoint)
            .unwrap()
    );
    assert_eq!(model.to_json().unwrap(), text);
}
#[test]
fn selected_queries_replay_in_a_fresh_model_process_and_split_again() {
    const INPUT: &str = "HYPERBREP_SELECTED_QUERY_ARCHIVE";
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let text = model().to_json().unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-query-{}-{}.json",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_queries_replay_in_a_fresh_model_process_and_split_again",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
#[test]
fn selected_query_model_rejects_original_poles_and_invalid_selectors() {
    let text = model().to_json().unwrap();
    let raw: Value = serde_json::from_str(&text).unwrap();
    for mode in 0..3 {
        let mut bad = raw.clone();
        match mode {
            0 => {
                bad["pcurves"][0]["Chord"]["start"]["RationalQueryPair"]["denominators"][0] =
                    json!([[Real::zero()]])
            }
            1 => {
                bad["pcurves"][0]["Chord"]["start"]["RationalQueryPair"]["sources"][0]["PolynomialFiber"]
                    ["interval"] = json!([Real::from(3), Real::from(4)])
            }
            _ => {
                let query = bad["selection"]["parameters"]
                    .as_array_mut()
                    .unwrap()
                    .iter_mut()
                    .find(|p| p.get("RationalForm").is_some())
                    .unwrap();
                query["RationalForm"]["denominator"] = json!([[Real::zero()]]);
            }
        }
        let error = RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
            .and_then(RawModel::validate)
            .unwrap_err();
        assert!(matches!(error,
            PersistenceError::Build(BuildError::Geometry(GeometryError::PlanarCurve(
                hypercurve::ExactCurveError::Invalid { cause, .. }
            ))) if mode == 1 || cause == hypercurve::CurveError::ZeroRationalPointDenominator
        ));
        verify(&text);
    }
}
#[test]
fn selected_query_archive_metadata_limits_count_sources_and_polynomials() {
    for scalar in [false, true] {
        let curve = if scalar {
            let point: CurvePoint2 = Uv::from_values(0, 1).into();
            let parameter = point
                .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[2]]))
                .unwrap()
                .parameter()
                .unwrap();
            let source: Curve2 =
                hypercurve::LineSeg2::try_new(Uv::from_values(0, 0), Uv::from_values(1, 0))
                    .unwrap()
                    .into();
            source
                .restrict_source(
                    CurveParameterRange2::try_new(CurveParameter2::from(Real::zero()), parameter)
                        .unwrap(),
                )
                .unwrap()
        } else {
            let point = pair([Uv::from_values(0, 2).into(), Uv::from_values(0, 3).into()]);
            Curve2::try_line(point, Uv::from_values(4, 3).into()).unwrap()
        };
        let mut builder = ModelBuilder::new();
        builder.pcurve(Pcurve::new(curve)).unwrap();
        let text = builder.finish().unwrap().to_json().unwrap();
        let (records, elements) = if scalar { (4, 5) } else { (5, 10) };
        let exact = ArchiveLimits {
            records,
            degree: 1,
            geometry_elements: elements,
            topology_references: 0,
            ..ArchiveLimits::default()
        };
        let restored = RawModel::from_json(&text, &exact)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(restored.to_json().unwrap(), text);
        let mut malformed: Value = serde_json::from_str(&text).unwrap();
        if scalar {
            malformed["selection"]["parameters"][0]["RationalForm"]["numerator"][0][0] =
                json!("invalid scalar");
        } else {
            malformed["pcurves"][0]["Chord"]["start"]["RationalQueryPair"]["numerators"][0][0][0] =
                json!("invalid scalar");
        }
        let malformed = malformed.to_string();
        assert!(RawModel::from_json(&malformed, &exact).is_err());
        for (limits, resource, observed) in [
            (
                ArchiveLimits {
                    records: records - 1,
                    ..exact
                },
                ArchiveResource::Records,
                records,
            ),
            (
                ArchiveLimits { degree: 0, ..exact },
                ArchiveResource::Degree,
                1,
            ),
            (
                ArchiveLimits {
                    geometry_elements: elements - 1,
                    ..exact
                },
                ArchiveResource::GeometryElements,
                elements,
            ),
        ] {
            for input in [&text, &malformed] {
                assert!(matches!(RawModel::from_json(input, &limits),
                    Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
                        resource: actual, observed: count, ..
                    })) if actual == resource && count == observed));
            }
            assert!(
                RawModel::from_json(&text, &exact)
                    .unwrap()
                    .validate()
                    .is_ok()
            );
        }
    }
}

#[test]
fn dependent_polynomial_fiber_replays_in_a_fresh_process_and_feeds_geometry() {
    const INPUT: &str = "HYPERBREP_DEPENDENT_POLYNOMIAL_FIBER_REPLAY";
    fn verify(text: &str) {
        let model = replay(text);
        let curve = model.pcurves().next().unwrap().curve().clone();
        let point = curve.start();
        for equation in [
            polynomial(&[&[-2], &[], &[], &[1]]),
            polynomial(&[&[0, 0, 1], &[-1]]),
        ] {
            assert_eq!(
                point
                    .rational_form(equation, polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap(),
                Ordering::Equal
            );
        }
        assert_eq!(
            point
                .affine_form(0.into(), 1.into(), 0.into())
                .compare_to_real(&Real::one())
                .unwrap(),
            Ordering::Greater
        );
        let root = point
            .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[1]]))
            .unwrap()
            .parameter()
            .unwrap();
        let next = root
            .affine_point(2.into(), 1.into(), 3.into(), (-1).into())
            .unwrap();
        assert_eq!(
            next.rational_form(polynomial(&[&[-5, -2], &[3]]), polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(model.to_json().unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let parameter = fiber(2)
        .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[1]]))
        .unwrap()
        .parameter()
        .unwrap();
    let point = CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[0, 0, 1], &[-1]]),
        parameter,
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let end = point
        .rational_image(&RationalMap2::new(
            [polynomial(&[&[1], &[1]]), polynomial(&[&[0, 1]])],
            polynomial(&[&[1]]),
        ))
        .unwrap();
    let mut builder = ModelBuilder::new();
    builder
        .pcurve(Pcurve::new(Curve2::try_line(point, end).unwrap()))
        .unwrap();
    let text = builder.finish().unwrap().to_json().unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-dependent-fiber-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "dependent_polynomial_fiber_replays_in_a_fresh_process_and_feeds_geometry",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
