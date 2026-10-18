use hyperbrep::{
    ArchiveLimits, ArchiveResource, Model, ModelBuilder, OperationContext, Parameter, Pcurve,
    PersistenceError, RawModel, Real, WorkBudget,
};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParallel2, BezierParallelSource2, BezierParameter2,
    BezierParameterInterval, BezierParameterPolynomial, BezierParameterRange2, CubicBezier2,
    Curve2, CurveCertainty, CurveParameterRange2, CurvePath2, CurveRegion2, CurveRegionLoopRole,
    FillRule, HomogeneousControl2, LineSeg2, Point2, PredicatePolicy, QuadraticBezier2,
    RationalBezier2, RegionPointLocation, Similarity2,
};
use serde_json::{Value, json};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn arch() -> BezierParallel2 {
    QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 2),
        Point2::from_values(4, 0),
    )
    .parallel_left(Real::one())
    .unwrap()
}

fn curve(parallel: BezierParallel2, a: BezierParameter2, b: BezierParameter2) -> Curve2 {
    Curve2::try_analytic_parallel(parallel, BezierParameterRange2::try_new(a, b).unwrap()).unwrap()
}

fn span(parallel: BezierParallel2, a: Real, b: Real) -> Curve2 {
    curve(
        parallel,
        BezierParameter2::Exact(a),
        BezierParameter2::Exact(b),
    )
}

pub fn model(curves: Vec<Curve2>) -> Model {
    let mut builder = ModelBuilder::new();
    for curve in curves {
        builder.pcurve(Pcurve::new(curve)).unwrap();
    }
    builder.finish().unwrap()
}

fn replay(source: &Model) -> Model {
    let json = source.to_json().unwrap();
    let outcome = hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
        RawModel::from_json(&json, &ArchiveLimits::default())?.validate()
    });
    assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    let restored = outcome.into_unverified().unwrap();
    assert_eq!(restored.counts(), source.counts());
    assert_eq!(restored.to_json().unwrap(), json);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(directory) = std::env::var_os("HYPERBREP_PARALLEL_ARCHIVE_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        let index = NEXT.fetch_add(1, Ordering::Relaxed);
        std::fs::write(directory.join(format!("replay-{index:03}.json")), &json).unwrap();
    }
    restored
}

fn same_definition(original: &Curve2, restored: &Curve2) {
    let a = original.analytic_parallel().unwrap();
    let b = restored.analytic_parallel().unwrap();
    assert_eq!(a.parallel().source(), b.parallel().source());
    assert_eq!(a.parallel().distance(), b.parallel().distance());
    assert_eq!(a.is_reversed(), b.is_reversed());
    for (a, b) in [
        (a.range().start(), b.range().start()),
        (a.range().end(), b.range().end()),
    ] {
        assert_eq!(
            a.as_bezier_parameter()
                .unwrap()
                .cmp_by_refinement(b.as_bezier_parameter().unwrap())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(a.scalar().is_some(), b.scalar().is_some());
    }
}

pub fn nonlinear_parallel_pcurve_replays_exactly() {
    for (a, b) in [(0, 1), (1, 0), (-1, 2), (2, -1)] {
        let original = span(arch(), a.into(), b.into());
        let stored = model(vec![original.clone()]);
        let restored = replay(&stored);
        let pcurve = restored.pcurves().next().unwrap().geometry();
        same_definition(&original, pcurve.curve());
        // Independently: P(t)=(4t,4t(1-t)); its left unit normal is
        // (-(1-2t),1)/sqrt(1+(1-2t)^2). These are exact interior values.
        let root = Real::from(5).sqrt().unwrap();
        for (t, expected) in [
            (
                q(1, 4),
                Point2::new(
                    Real::one() - (Real::one() / &root).unwrap(),
                    q(3, 4) + (Real::from(2) / root).unwrap(),
                ),
            ),
            (q(1, 2), Point2::from_values(2, 2)),
        ] {
            assert!(
                pcurve
                    .point_at(&Parameter::from_real(t))
                    .unwrap()
                    .coincides_with(&expected.into())
                    .unwrap()
            );
        }
        let twice = pcurve.reversed().unwrap().reversed().unwrap();
        same_definition(pcurve.curve(), twice.curve());
    }
}

pub fn polynomial_and_rational_parallel_sources_replay() {
    let p = Point2::from_values;
    let homogeneous = RationalBezier2::from_homogeneous_controls(vec![
        HomogeneousControl2::new(1.into(), 0.into(), 1.into()),
        HomogeneousControl2::new(0.into(), 1.into(), 0.into()),
        HomogeneousControl2::new((-1).into(), 0.into(), 1.into()),
    ])
    .unwrap();
    let sources = vec![
        (
            BezierParallelSource2::Cubic(CubicBezier2::new(p(0, 0), p(1, 1), p(3, 1), p(4, 0))),
            Real::one(),
        ),
        (
            BezierParallelSource2::Rational(
                RationalBezier2::try_new(
                    vec![p(0, 0), p(2, 2), p(4, 0)],
                    vec![1.into(), 2.into(), 1.into()],
                )
                .unwrap(),
            ),
            Real::one(),
        ),
        (BezierParallelSource2::Rational(homogeneous), q(1, 4)),
        (
            BezierParallelSource2::Rational(
                RationalBezier2::try_new(
                    (0..=7).map(|i| p(i, i * i)).collect(),
                    vec![Real::one(); 8],
                )
                .unwrap(),
            ),
            Real::zero(),
        ),
        (
            BezierParallelSource2::Quadratic(QuadraticBezier2::new(p(0, 0), p(0, 0), p(0, 0))),
            Real::zero(),
        ),
    ];
    for (source, distance) in sources {
        let original = span(
            BezierParallel2::from_source(source, distance),
            0.into(),
            1.into(),
        );
        let rebuilt = replay(&model(vec![original.clone(), original.reversed().unwrap()]));
        for (expected, restored) in [original.clone(), original.reversed().unwrap()]
            .iter()
            .zip(rebuilt.pcurves())
        {
            same_definition(expected, restored.geometry().curve());
            for t in [q(1, 4), q(1, 2), q(3, 4)] {
                assert!(
                    expected
                        .point_at(&t.clone().into())
                        .unwrap()
                        .coincides_with(&restored.geometry().point_at(&t.into()).unwrap())
                        .unwrap()
                );
            }
        }
    }
}

fn selected() -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
}

pub fn selected_parallel_ranges_survive_replay_and_subdivision() {
    let selected = selected();
    assert!(selected.scalar().is_none());
    let transported = selected.affine_image(&q(1, 2), &q(1, 4)).unwrap();
    let original = curve(
        arch(),
        BezierParameter2::Exact(q(1, 8)),
        transported.clone(),
    );
    let source = model(vec![original.clone(), original.reversed().unwrap()]);
    let json: Value = serde_json::from_str(&source.to_json().unwrap()).unwrap();
    assert_eq!(json["selection"]["parameters"].as_array().unwrap().len(), 2);
    let rebuilt = replay(&source);
    for (expected, restored) in [original.clone(), original.reversed().unwrap()]
        .iter()
        .zip(rebuilt.pcurves())
    {
        same_definition(expected, restored.geometry().curve());
        let point = restored
            .geometry()
            .point_at(&restored.geometry().domain_end().unwrap())
            .unwrap();
        assert!(point.coincides_with(&expected.end()).unwrap());
    }
    let split_parameter = rebuilt
        .pcurves()
        .next()
        .unwrap()
        .geometry()
        .domain_end()
        .unwrap();
    assert!(split_parameter.try_real().is_none());
    for reversed in [false, true] {
        let complete = Pcurve::new(span(
            arch(),
            if reversed { 1.into() } else { 0.into() },
            if reversed { 0.into() } else { 1.into() },
        ));
        let (left, right) = complete.split_at(&split_parameter).unwrap();
        let parts = replay(&model(vec![left.curve().clone(), right.curve().clone()]));
        let curves = parts
            .pcurves()
            .map(|c| c.geometry().curve().clone())
            .collect::<Vec<_>>();
        assert!(curves[0].end().coincides_with(&curves[1].start()).unwrap());
        assert!(
            curves[0]
                .end()
                .coincides_with(&complete.point_at(&split_parameter).unwrap())
                .unwrap()
        );
        assert!(selected.scalar().is_none());
        assert!(transported.scalar().is_none());
    }
}

fn cusp_parts() -> Vec<Curve2> {
    let parallel = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(q(1, 2), 0.into()),
        Point2::from_values(1, 1),
    )
    .parallel_left(Real::one())
    .unwrap();
    let analysis = parallel
        .singularity_analysis(&CurveParameterRange2::unit())
        .unwrap();
    let [cusp] = analysis.parallel_cusps() else {
        panic!("one exact cusp");
    };
    vec![
        curve(
            parallel.clone(),
            BezierParameter2::Exact(Real::zero()),
            cusp.clone(),
        ),
        curve(parallel, cusp.clone(), BezierParameter2::Exact(Real::one())),
    ]
}

pub fn parallel_cusp_endpoints_remain_selected_on_replay() {
    let curves = cusp_parts();
    let restored = replay(&model(curves.clone()));
    let pcurves = restored.pcurves().map(|p| p.geometry()).collect::<Vec<_>>();
    for (original, restored) in curves.iter().zip(&pcurves) {
        same_definition(original, restored.curve());
    }
    assert!(pcurves[0].domain_end().unwrap().try_real().is_none());
    assert!(
        pcurves[0]
            .curve()
            .end()
            .coincides_with(&pcurves[1].curve().start())
            .unwrap()
    );
}

pub fn cap_curves() -> Vec<Curve2> {
    let parallel = arch();
    let left = parallel.point_at(&Real::zero()).unwrap();
    let right = parallel.point_at(&Real::one()).unwrap();
    let lower_left = Point2::new(left.x().clone(), (-2).into());
    let lower_right = Point2::new(right.x().clone(), (-2).into());
    let mut curves = vec![span(parallel, 1.into(), 0.into())];
    curves.extend(
        [
            (left, lower_left.clone()),
            (lower_left, lower_right.clone()),
            (lower_right, right),
        ]
        .into_iter()
        .map(|(a, b)| Curve2::from(LineSeg2::try_new(a, b).unwrap())),
    );
    curves
}

pub fn replayed_parallel_pcurves_reenter_planar_modeling() {
    let reflection = Similarity2::try_from_real_affine(
        (-2).into(),
        0.into(),
        0.into(),
        2.into(),
        7.into(),
        (-1).into(),
    )
    .unwrap();
    for transformed in [false, true] {
        let curves = cap_curves()
            .into_iter()
            .map(|c| {
                if transformed {
                    c.transform_similarity(&reflection).unwrap()
                } else {
                    c
                }
            })
            .collect();
        let restored = replay(&model(curves));
        let path = CurvePath2::try_new(
            restored
                .pcurves()
                .map(|p| p.geometry().curve().clone())
                .collect(),
        )
        .unwrap();
        for path in [path.clone(), path.reversed().unwrap()] {
            let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[path],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
            )
            .unwrap();
            let tiny: Real = "1e-12".parse().unwrap();
            for (y, expected) in [
                (Real::zero(), RegionPointLocation::Inside),
                (Real::from(2), RegionPointLocation::Boundary),
                (Real::from(2) - &tiny, RegionPointLocation::Inside),
                (Real::from(2) + &tiny, RegionPointLocation::Outside),
            ] {
                let point = Point2::new(2.into(), y);
                let point = if transformed {
                    reflection.transform_point(&point)
                } else {
                    point
                };
                assert_eq!(region.classify_point(&point.into()).unwrap(), expected);
            }
        }
    }
}

fn parsed(source: &Model) -> Value {
    serde_json::from_str(&source.to_json().unwrap()).unwrap()
}
fn decode(value: &Value) -> Result<Model, PersistenceError> {
    RawModel::from_json(&value.to_string(), &ArchiveLimits::default())?.validate()
}
fn scalar(value: i32) -> Value {
    json!({"Scalar": Real::from(value)})
}

pub fn parallel_archives_reject_forged_definitions() {
    let good = model(vec![span(arch(), 0.into(), 1.into())]);
    for mode in 0..3 {
        let mut raw = parsed(&good);
        let parallel = &mut raw["pcurves"][0]["AnalyticParallel"];
        match mode {
            0 => parallel["range"] = json!([scalar(1), scalar(0)]),
            1 => parallel["range"] = json!([scalar(0), scalar(0)]),
            _ => {
                let point = parallel["source"]["Quadratic"][0].clone();
                parallel["source"]["Quadratic"] = json!([point, point, point]);
            }
        }
        assert!(
            matches!(decode(&raw), Err(PersistenceError::Build(_))),
            "{mode}"
        );
        replay(&good);
    }
    let mut raw = parsed(&model(cusp_parts()));
    raw["pcurves"][0]["AnalyticParallel"]["range"] = json!([scalar(0), scalar(1)]);
    assert!(
        matches!(decode(&raw), Err(PersistenceError::Build(_))),
        "an unsplit interior cusp cannot borrow the endpoint proof"
    );
    let selected = curve(arch(), BezierParameter2::Exact(Real::zero()), selected());
    let mut raw = parsed(&model(vec![selected]));
    raw["pcurves"][0]["AnalyticParallel"]["range"][1] = json!({"Selected":1000000});
    assert!(matches!(
        decode(&raw),
        Err(PersistenceError::InvalidReference {
            kind: "selected parameter",
            index: 1000000
        })
    ));
    let mut raw = parsed(&good);
    // W(t)=1-2t: the source is undefined at 1/2, even at zero offset.
    raw["pcurves"][0]["AnalyticParallel"]["source"] = json!({"Rational":{"homogeneous_controls":[[Real::zero(),Real::zero(),Real::one()],[Real::one(),Real::zero(),-Real::one()]]}});
    raw["pcurves"][0]["AnalyticParallel"]["distance"] = json!(Real::zero());
    assert!(
        matches!(decode(&raw), Err(PersistenceError::Build(_))),
        "zero distance must not hide a pole"
    );
    replay(&good);
}

pub fn parallel_archive_limits_and_exhaustion_recover() {
    let good = model(vec![span(arch(), 0.into(), 1.into())]);
    let json = good.to_json().unwrap();
    for (limits, resource, observed) in [
        (
            ArchiveLimits {
                records: 1,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Records,
            2,
        ),
        (
            ArchiveLimits {
                degree: 1,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Degree,
            2,
        ),
        (
            ArchiveLimits {
                geometry_elements: 2,
                ..ArchiveLimits::default()
            },
            ArchiveResource::GeometryElements,
            3,
        ),
    ] {
        let Err(PersistenceError::ResourceLimit(error)) = RawModel::from_json(&json, &limits)
        else {
            panic!("source admission must fail typed")
        };
        assert_eq!(error.resource, resource);
        assert_eq!(error.observed, observed);
        replay(&good);
    }
    let mut malformed = parsed(&good);
    malformed["pcurves"][0]["AnalyticParallel"]["source"]["Quadratic"][0][0] =
        json!("not a scalar");
    let malformed = malformed.to_string();
    assert!(matches!(
        RawModel::from_json(
            &malformed,
            &ArchiveLimits {
                degree: 1,
                ..ArchiveLimits::default()
            }
        ),
        Err(PersistenceError::ResourceLimit(_))
    ));
    assert!(matches!(
        RawModel::from_json(&malformed, &ArchiveLimits::default()),
        Err(PersistenceError::Json(_))
    ));
    let selected = model(vec![curve(
        arch(),
        BezierParameter2::Exact(Real::zero()),
        selected(),
    )]);
    let json = selected.to_json().unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    // This isolator and the cusp-free source need no instrumented refinement.
    // A zero budget must still admit those already sufficient exact proofs.
    assert_eq!(denied.result.unwrap().unwrap().to_json().unwrap(), json);
    let cusp_json = model(cusp_parts()).to_json().unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&cusp_json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    assert!(
        denied.result.is_err(),
        "cold cusp replay must charge root isolation"
    );
    RawModel::from_json(&cusp_json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    replay(&selected);
    let provisional =
        hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || replay(&selected));
    assert_eq!(provisional.certainty(), CurveCertainty::Certified);
    replay(&provisional.into_unverified());
}
