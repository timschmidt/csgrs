use hyperbrep::{
    ArchiveLimits, ArchiveResource, Model, ModelBuilder, Pcurve, PersistenceError, RawModel,
};
use hypercurve::{
    Curve2, CurvePoint2, CurvePointDefinition2, CurvePointRationalForm2, Point2, Real,
};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use serde_json::Value;
use std::cmp::Ordering;

fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn form(point: &CurvePoint2, numerator: BivariatePolynomial) -> CurvePointRationalForm2 {
    point.rational_form(numerator, p(&[&[1]])).unwrap()
}
fn select(coefficients: Vec<CurvePointRationalForm2>, exact: bool) -> CurvePoint2 {
    CurvePointDefinition2::SelectedPolynomialRoot {
        coefficients,
        root: IsolatedRootInterval {
            lower: if exact { 1.into() } else { 0.into() },
            upper: if exact { 1.into() } else { 2.into() },
            exact_root: exact.then(Real::one),
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}
fn model(point: CurvePoint2) -> Model {
    let mut builder = ModelBuilder::new();
    builder
        .pcurve(Pcurve::new(
            Curve2::try_line(point, Point2::from_values(3, 1).into()).unwrap(),
        ))
        .unwrap();
    builder.finish().unwrap()
}
fn replay(text: &str) -> Model {
    RawModel::from_json(text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap()
}
fn root_model() -> Model {
    let source = CurvePointDefinition2::PolynomialFiber {
        equation: p(&[&[0, 0, 0, 1], &[-1]]),
        parameter: Real::from(2).into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    model(select(
        vec![
            form(&source, p(&[&[0, -1]])),
            form(&source, p(&[&[0]])),
            form(&source, p(&[&[1]])),
        ],
        false,
    ))
}
fn verify(text: &str) {
    let cold = replay(text);
    let curve = cold.pcurves().next().unwrap();
    let point = curve.curve().start();
    assert_eq!(
        form(&point, p(&[&[-2], &[], &[], &[], &[], &[], &[1]]))
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        form(&point, p(&[&[], &[1]]))
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Greater
    );
    let parameter = form(&point, p(&[&[], &[1]])).parameter().unwrap();
    let next = parameter
        .affine_point(2.into(), 1.into(), 3.into(), (-1).into())
        .unwrap();
    assert_eq!(
        form(&next, p(&[&[-5, -2], &[3]]))
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        curve
            .curve()
            .reversed()
            .unwrap()
            .end()
            .coincides_with(&point)
            .unwrap()
    );
    assert_eq!(cold.to_json().unwrap(), text);
}
#[test]
fn selected_coefficient_root_archive_replays_cold_and_feeds_geometry() {
    const INPUT: &str = "HYPERBREP_SELECTED_COEFFICIENT_ROOT_REPLAY";
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let text = root_model().to_json().unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-coefficient-roots-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_coefficient_root_archive_replays_cold_and_feeds_geometry",
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
fn root_record(value: &mut Value) -> Option<&mut Value> {
    match value {
        Value::Object(map) => {
            if map.contains_key("SelectedPolynomialRoot") {
                return map.get_mut("SelectedPolynomialRoot");
            }
            map.values_mut().find_map(root_record)
        }
        Value::Array(array) => array.iter_mut().find_map(root_record),
        _ => None,
    }
}
#[test]
fn selected_coefficient_root_archives_reject_bad_references_selectors_and_guards() {
    let text = root_model().to_json().unwrap();
    let raw: Value = serde_json::from_str(&text).unwrap();
    for mode in 0..4 {
        let mut bad = raw.clone();
        match mode {
            0 => {
                root_record(&mut bad).unwrap()["coefficients"][0] =
                    serde_json::json!({"Selected":999999})
            }
            1 => {
                root_record(&mut bad).unwrap()["coefficients"][0] =
                    serde_json::json!({"Scalar":Real::zero()})
            }
            2 => {
                root_record(&mut bad).unwrap()["interval"] =
                    serde_json::json!([Real::from(3), Real::from(4)])
            }
            _ => {
                let query = bad["selection"]["parameters"]
                    .as_array_mut()
                    .unwrap()
                    .iter_mut()
                    .find(|v| v.get("RationalForm").is_some())
                    .unwrap();
                query["RationalForm"]["denominator"] = serde_json::json!([[Real::zero()]]);
            }
        }
        assert!(
            RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .is_err()
        );
    }
    assert!(
        matches!(RawModel::from_json(&text, &ArchiveLimits { degree: 1, ..ArchiveLimits::default() }), Err(PersistenceError::ResourceLimit(limit)) if limit.resource == ArchiveResource::Degree)
    );
}
#[test]
fn selected_coefficient_root_archives_share_repeated_dependencies() {
    let mut sizes = Vec::new();
    for depth in [6, 12] {
        let mut point: CurvePoint2 = Point2::from_values(1, 0).into();
        for _ in 0..depth {
            point = select(
                vec![
                    form(&point, p(&[&[], &[-1]])),
                    form(&point, p(&[&[], &[1]])),
                ],
                true,
            );
        }
        let text = model(point).to_json().unwrap();
        let raw: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(
            raw["selection"]["parameters"].as_array().unwrap().len(),
            3 * depth + 3
        );
        let cold = replay(&text);
        let point = cold.pcurves().next().unwrap().curve().start();
        assert_eq!(
            form(&point, p(&[&[], &[1]]))
                .compare_to_real(&Real::one())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(cold.to_json().unwrap(), text);
        sizes.push(text.len());
    }
    assert!(
        sizes[1] < 3 * sizes[0],
        "doubling shared definition depth must not expand a binary expression tree: {sizes:?}"
    );
}

#[test]
fn selected_coefficient_dependency_replay_export_and_drop_on_small_stack() {
    const INPUT: &str = "HYPERBREP_SMALL_STACK_ROOT_REPLAY";
    if let Some(path) = std::env::var_os(INPUT) {
        let text = std::fs::read_to_string(path).unwrap();
        std::thread::Builder::new()
            .stack_size(256 * 1024)
            .spawn(move || {
                let raw = RawModel::from_json(&text, &ArchiveLimits::default()).unwrap();
                assert_eq!(raw.to_json().unwrap(), text);
                let cold = raw.validate().unwrap();
                assert_eq!(cold.to_json().unwrap(), text);
                let point = cold.pcurves().next().unwrap().curve().start();
                assert_eq!(
                    form(&point, p(&[&[], &[1]]))
                        .compare_to_real(&Real::one())
                        .unwrap(),
                    Ordering::Equal
                );
                drop(point);
                drop(cold);
            })
            .unwrap()
            .join()
            .unwrap();
        return;
    }
    // Keep the original shared-dependency workload; flat parameter references
    // conceal its logical depth from the JSON container-depth limit.
    let mut point: CurvePoint2 = Point2::from_values(1, 0).into();
    for _ in 0..12 {
        point = select(
            vec![
                form(&point, p(&[&[], &[-1]])),
                form(&point, p(&[&[], &[1]])),
            ],
            true,
        );
    }
    let text = model(point).to_json().unwrap();
    if let Some(path) = std::env::var_os("HYPERBREP_DEPENDENCY_EVIDENCE") {
        std::fs::write(path, &text).unwrap();
    }

    let path =
        std::env::temp_dir().join(format!("hyperbrep-small-stack-{}.json", std::process::id()));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_coefficient_dependency_replay_export_and_drop_on_small_stack",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "small-stack dependency replay failed: {}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

fn flat_dependency_document(depth: usize) -> String {
    let mut raw: Value =
        serde_json::from_str(&model(Point2::from_values(1, 0).into()).to_json().unwrap()).unwrap();
    let mut point = serde_json::json!({"Coordinates":[Real::one(),Real::zero()]});
    let mut definitions = Vec::new();
    for level in 0..depth {
        for sign in [-1, 1] {
            definitions.push(serde_json::json!({"RationalForm": {
                "point":point.clone(),"numerator":[[],[Real::from(sign)]],"denominator":[[Real::one()]]
            }}));
        }
        point = serde_json::json!({"SelectedPolynomialRoot": {
            "coefficients":[{"Selected":2*level},{"Selected":2*level+1}],
            "interval":[Real::one(),Real::one()],"exact_root":Real::one()
        }});
    }
    raw["selection"]["parameters"] = Value::Array(definitions);
    raw["pcurves"] = serde_json::json!([{"Chord":{"start":point,"end":{"Coordinates":[Real::from(3),Real::one()]}}}]);
    raw.to_string()
}

#[test]
fn flat_selected_dependency_depth_precedes_scalar_replay_and_allows_retry() {
    let text = flat_dependency_document(64);
    let mut bad: Value = serde_json::from_str(&text).unwrap();
    bad["selection"]["parameters"][0]["RationalForm"]["numerator"] =
        serde_json::json!([["invalid exact scalar"]]);
    for input in [text.clone(), bad.to_string()] {
        std::thread::Builder::new().stack_size(256*1024).spawn(move || {
            let error = RawModel::from_json(&input,&ArchiveLimits { nesting_depth: 11,..ArchiveLimits::default() }).unwrap_err();
            assert!(matches!(error,PersistenceError::ResourceLimit(ref reached) if reached.resource==ArchiveResource::DependencyDepth && reached.limit==64 && reached.observed==65),"{error:?}");
        }).unwrap().join().unwrap();
    }
    // A larger allowance on an ordinary stack preserves the original input.
    // This is a dependency limit, not a promise about arbitrary stack sizes.
    std::thread::Builder::new()
        .stack_size(2 * 1024 * 1024)
        .spawn(move || {
            let limits = ArchiveLimits {
                dependency_depth: 256,
                ..ArchiveLimits::default()
            };
            let cold = RawModel::from_json(&text, &limits)
                .unwrap()
                .validate()
                .unwrap();
            let point = cold.pcurves().next().unwrap().curve().start();
            assert_eq!(
                form(&point, p(&[&[], &[1]]))
                    .compare_to_real(&Real::one())
                    .unwrap(),
                Ordering::Equal
            );
            let canonical = cold.to_json().unwrap();
            let again = RawModel::from_json(&canonical, &limits)
                .unwrap()
                .validate()
                .unwrap();
            assert_eq!(again.to_json().unwrap(), canonical);
            drop(point);
            drop(cold);
            drop(again);
        })
        .unwrap()
        .join()
        .unwrap();
}

#[test]
fn shared_definition_references_reject_cycles_forward_and_wrong_types() {
    let text = root_model().to_json().unwrap();
    let raw: Value = serde_json::from_str(&text).unwrap();
    let nodes = raw["selection"]["parameters"].as_array().unwrap();
    let point = nodes
        .iter()
        .rposition(|v| v.get("PlanarPoint").is_some())
        .unwrap();
    let parameter = nodes
        .iter()
        .position(|v| v.get("RationalForm").is_some())
        .unwrap();
    assert!(parameter < point);
    for reference in [point, nodes.len(), parameter] {
        let mut bad = raw.clone();
        bad["selection"]["parameters"][point] =
            serde_json::json!({"PlanarPoint":{"Reference":reference}});
        assert!(
            matches!(RawModel::from_json(&bad.to_string(),&ArchiveLimits::default()).unwrap().validate(),Err(PersistenceError::InvalidReference{kind:"planar point",index}) if index==reference)
        );
    }
    let mut bad = raw.clone();
    bad["pcurves"][0] = serde_json::json!({"ChordReference":parameter});
    assert!(
        matches!(RawModel::from_json(&bad.to_string(),&ArchiveLimits::default()).unwrap().validate(),Err(PersistenceError::InvalidReference{kind:"chord",index}) if index==parameter)
    );
    let first_point = nodes
        .iter()
        .position(|v| v.get("PlanarPoint").is_some())
        .unwrap();
    let mut bad = raw.clone();
    root_record(&mut bad).unwrap()["coefficients"][0] = serde_json::json!({"Selected":first_point});
    assert!(
        matches!(RawModel::from_json(&bad.to_string(),&ArchiveLimits::default()).unwrap().validate(),Err(PersistenceError::InvalidReference{kind:"selected parameter",index}) if index==first_point)
    );
    verify(&text);
}
