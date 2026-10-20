use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameter2, CurveParameterRange2, PolynomialCurveSet2,
    Similarity2,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, BivariateTopologyError,
    decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use serde_json::{Value, json};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn curves(equation: Vec<Vec<Real>>, domain: [[Real; 2]; 2]) -> PolynomialCurveSet2 {
    let [first, second] = domain;
    let config = BivariateTopologyConfig {
        max_refinement_rounds: 97,
        ..Default::default()
    };
    let topology = decompose_bivariate_curve(
        &BivariatePolynomial::new(equation),
        &BivariateCurveDomain { first, second },
        config,
    )
    .unwrap();
    assert_eq!(topology.config(), config);
    PolynomialCurveSet2::from_topology(
        topology,
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 93,
            refinement_steps: 9,
        },
    )
    .unwrap()
}

fn circle() -> PolynomialCurveSet2 {
    curves(
        vec![
            vec![q(7, 16), (-1).into(), 1.into()],
            vec![(-1).into()],
            vec![1.into()],
        ],
        [[0.into(), 1.into()], [0.into(), 1.into()]],
    )
}

fn selected_sqrt_two() -> CurveParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![(-2).into(), 0.into(), 1.into()])
                .unwrap(),
            BezierParameterInterval::try_new(1.into(), 2.into()).unwrap(),
        )
        .unwrap(),
    )
    .into()
}

fn model(sources: &[Curve2]) -> Model {
    let mut builder = ModelBuilder::new();
    for source in sources {
        builder.pcurve(Pcurve::new(source.clone())).unwrap();
        let curve = Curve3::planar_image(
            source.clone(),
            Point3::new(2.into(), (-1).into(), 4.into()),
            Vector3::new([2.into(), 1.into(), 0.into()]),
            Vector3::new([(-1).into(), 3.into(), 1.into()]),
        )
        .unwrap();
        let a = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let b = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let domain = curve.domain().clone();
        let curve = builder.curve(curve).unwrap();
        builder.edge(&a, &b, &curve, domain).unwrap();
    }
    builder.finish().unwrap()
}

fn replay(model: &Model, tag: &str) -> Model {
    let archive = model.to_json().unwrap();
    if let Some(directory) = std::env::var_os("HYPERBREP_POLYNOMIAL_REPLAY_ARTIFACT_DIR") {
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(
            std::path::Path::new(&directory).join(format!("{tag}.json")),
            &archive,
        )
        .unwrap();
    }
    let restored = RawModel::from_json(&archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.counts(), model.counts());
    assert_eq!(restored.to_json().unwrap(), archive);
    for (a, b) in model.curves().zip(restored.curves()) {
        let (a, b) = (a.geometry(), b.geometry());
        assert_eq!(a.kind(), b.kind());
        for parameter in [a.domain().start(), a.domain().end()] {
            assert!(
                a.point_at(parameter)
                    .unwrap()
                    .coincides_with(&b.point_at(parameter).unwrap())
                    .unwrap()
            );
        }
        b.bounds().unwrap();
    }
    restored
}

fn source(curve: &Curve2) -> &Curve2 {
    curve
        .source_restriction()
        .map_or(curve, |(source, _, _)| source)
}

fn inspect_sources(model: &Model) {
    let branches = model
        .pcurves()
        .map(|p| source(p.geometry().curve()).polynomial_branch().unwrap())
        .collect::<Vec<_>>();
    for branch in &branches {
        assert!(branches[0].shares_source(branch));
        assert_eq!(branch.topology().config().max_refinement_rounds, 97);
        assert_eq!(branch.evaluation_config().max_subdivision_depth, 93);
        assert_eq!(branch.evaluation_config().refinement_steps, 9);
    }
    for curve in model.curves() {
        let branch = source(curve.geometry().planar_geometry().unwrap().source)
            .polynomial_branch()
            .unwrap();
        assert!(branches[0].shares_source(branch));
    }
}

pub fn complete_branches_compose_and_replay() {
    let set = circle();
    assert_eq!(set.curves().len(), 2);
    let similarity = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let mut originals = Vec::new();
    for curve in set.curves() {
        let (left, right) = curve.split_at(q(1, 2).into()).unwrap();
        for curve in [curve.clone(), curve.reversed().unwrap(), left, right] {
            originals.push(curve.transform_similarity(&similarity).unwrap());
        }
    }
    let first = model(&originals);
    let raw: Value = serde_json::from_str(&first.to_json().unwrap()).unwrap();
    assert_eq!(
        raw["selection"]["polynomial_sources"]
            .as_array()
            .unwrap()
            .len(),
        1
    );
    let mut restored = first;
    for i in 0..3 {
        restored = replay(&restored, &format!("circle-{i}"));
        inspect_sources(&restored);
    }
    for (index, curve) in restored.pcurves().enumerate() {
        let point = curve.geometry().curve().point_at(&q(1, 2).into()).unwrap();
        let y = if index < 4 { q(1, 4) } else { q(3, 4) };
        let expected = similarity.transform_point(&hypercurve::Point2::new(q(1, 2), y));
        assert!(point.coincides_with(&expected.into()).unwrap());
    }
    let shear = Matrix4::from_row_major([
        (-1).into(),
        2.into(),
        0.into(),
        4.into(),
        0.into(),
        1.into(),
        0.into(),
        (-2).into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let transformed = restored.transformed(&shear).unwrap();
    let again = replay(&transformed, "circle-shear");
    inspect_sources(&again);
    for (a, b) in restored.vertices().zip(again.vertices()) {
        assert!(
            a.point()
                .transformed(&shear)
                .unwrap()
                .coincides_with(b.point())
                .unwrap()
        );
    }
}

pub fn selected_noncircular_restrictions_and_source_controls() {
    // y^3=x has a nonquadratic selected fiber over x=sqrt(2).
    let set = curves(
        vec![
            vec![0.into(), 0.into(), 0.into(), 1.into()],
            vec![(-1).into()],
        ],
        [[1.into(), 2.into()], [0.into(), 2.into()]],
    );
    let curve = &set.curves()[0];
    let parameter = selected_sqrt_two();
    let (left, right) = curve.split_at(parameter.clone()).unwrap();
    let range = CurveParameterRange2::try_new(Real::one().into(), parameter.clone()).unwrap();
    let restricted = curve.restrict_source(range).unwrap().reversed().unwrap();
    let original = model(&[curve.clone(), left, right, restricted]);
    let restored = replay(&replay(&original, "cubic-first"), "cubic-second");
    inspect_sources(&restored);
    for curve in restored.pcurves() {
        let point = curve.geometry().curve().point_at(&parameter).unwrap();
        assert!(point.coordinates().is_none());
        assert_eq!(
            point
                .rational_form(
                    BivariatePolynomial::new(vec![
                        vec![0.into(), 0.into(), 0.into(), 1.into()],
                        vec![(-1).into()]
                    ]),
                    BivariatePolynomial::new(vec![vec![1.into()]])
                )
                .unwrap()
                .compare_to_real(&0.into())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            point
                .affine_form(1.into(), 0.into(), 0.into())
                .parameter()
                .unwrap()
                .compare(&parameter)
                .unwrap(),
            Ordering::Equal
        );
    }
    for vertex in restored.vertices() {
        assert_eq!(
            vertex
                .point()
                .compare_linear_form(&Vector3::new([1.into(), (-2).into(), 7.into()]), &32.into())
                .unwrap(),
            Ordering::Equal
        );
    }
    // Point images are usable origins for subsequent exact straight sweeps.
    let mut builder = ModelBuilder::new();
    for vertex in restored.vertices() {
        let curve = Curve3::translation(vertex.point().clone(), Vector3::z()).unwrap();
        let a = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let b = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let domain = curve.domain().clone();
        let c = builder.curve(curve).unwrap();
        builder.edge(&a, &b, &c, domain).unwrap();
    }
    replay(&builder.finish().unwrap(), "cubic-sweep");
}

pub fn singular_disconnected_and_tiny_branches_replay() {
    let radius = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    let cases = [
        (
            vec![
                vec![0.into(), 0.into(), 1.into()],
                vec![],
                vec![(-1).into()],
            ],
            [[(-1).into(), 1.into()], [(-2).into(), 2.into()]],
            4,
        ),
        (
            vec![
                vec![0.into(), 0.into(), 1.into()],
                vec![],
                vec![],
                vec![(-1).into()],
            ],
            [[(-1).into(), 1.into()], [(-2).into(), 2.into()]],
            2,
        ),
        (
            vec![
                vec![(-2).into(), 0.into(), 1.into()],
                vec![],
                vec![1.into()],
            ],
            [[(-2).into(), 2.into()], [(-2).into(), 2.into()]],
            2,
        ),
        (
            vec![vec![(-1).into(), 0.into(), 1.into()]],
            [[0.into(), 1.into()], [(-2).into(), 2.into()]],
            0,
        ),
        (
            vec![
                vec![q(1, 2) - &radius * &radius, (-1).into(), 1.into()],
                vec![(-1).into()],
                vec![1.into()],
            ],
            [[0.into(), 1.into()], [0.into(), 1.into()]],
            0,
        ),
    ];
    for (index, (equation, domain, origin_incidence)) in cases.into_iter().enumerate() {
        let set = curves(equation, domain);
        let restored = replay(&model(set.curves()), &format!("graph-{index}"));
        inspect_sources(&restored);
        assert_eq!(restored.pcurves().len(), set.curves().len());
        if index < 2 {
            let count = restored
                .pcurves()
                .flat_map(|c| [c.geometry().curve().start(), c.geometry().curve().end()])
                .filter(|p| {
                    p.coincides_with(&hypercurve::Point2::from_values(0, 0).into())
                        .unwrap()
                })
                .count();
            assert_eq!(count, origin_incidence);
        }
        if index == 2 || index == 3 {
            // x^2+y^2=2 at x=1, and the two disconnected lines y^2=1.
            let x = if index == 2 { Real::one() } else { q(1, 2) };
            assert_eq!(restored.pcurves().len(), 2);
            for (side, curve) in restored.pcurves().enumerate() {
                let y = Real::from(if side == 0 { -1 } else { 1 });
                assert!(
                    curve
                        .geometry()
                        .curve()
                        .point_at(&x.clone().into())
                        .unwrap()
                        .coincides_with(&hypercurve::Point2::new(x.clone(), y).into())
                        .unwrap()
                );
            }
        }
        if index == 4 {
            for (side, curve) in restored.pcurves().enumerate() {
                let y = if side == 0 {
                    q(1, 2) - &radius
                } else {
                    q(1, 2) + &radius
                };
                assert!(
                    curve
                        .geometry()
                        .curve()
                        .point_at(&q(1, 2).into())
                        .unwrap()
                        .coincides_with(&hypercurve::Point2::new(q(1, 2), y).into())
                        .unwrap()
                );
            }
        }
    }
}

fn parsed(value: &Value) -> Result<Model, PersistenceError> {
    RawModel::from_json(&value.to_string(), &ArchiveLimits::default())?.validate()
}

pub fn untrusted_branch_definitions_and_predecode_limits() {
    let original: Value =
        serde_json::from_str(&model(circle().curves()).to_json().unwrap()).unwrap();
    for (key, index, kind) in [
        ("source", 99, "polynomial source"),
        ("slab", 99, "polynomial slab"),
        ("root", 99, "polynomial branch"),
    ] {
        let mut raw = original.clone();
        raw["pcurves"][0]["PolynomialBranch"][key] = index.into();
        assert_eq!(
            parsed(&raw).unwrap_err(),
            PersistenceError::InvalidReference { kind, index }
        );
    }
    let mut raw = original.clone();
    raw["pcurves"][0]["PolynomialBranch"]["range"][1] =
        raw["pcurves"][0]["PolynomialBranch"]["range"][0].clone();
    assert!(parsed(&raw).is_err());
    let mut raw = original.clone();
    raw["pcurves"][0]["PolynomialBranch"]["range"][0] =
        json!({"Scalar": serde_json::to_value(Real::zero()).unwrap()});
    assert!(parsed(&raw).is_err());
    let mut raw = original.clone();
    raw["pcurves"][0]["PolynomialBranch"]["range"]
        .as_array_mut()
        .unwrap()
        .swap(0, 1);
    assert!(parsed(&raw).is_err());
    let mut raw = original.clone();
    raw["pcurves"][0]["PolynomialBranch"]["transform"] =
        serde_json::to_value([1, 1, 0, 1, 0, 0].map(Real::from)).unwrap();
    assert!(parsed(&raw).is_err());
    let mut raw = original.clone();
    raw["selection"]["polynomial_sources"][0]["equation"] =
        serde_json::to_value(vec![vec![Real::zero()]]).unwrap();
    assert!(matches!(
        parsed(&raw),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PolynomialTopology(BivariateTopologyError::ZeroPolynomial)
        )))
    ));
    let mut raw = original.clone();
    raw["selection"]["polynomial_sources"][0]["domain"][0]
        .as_array_mut()
        .unwrap()
        .swap(0, 1);
    assert!(matches!(
        parsed(&raw),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PolynomialTopology(BivariateTopologyError::InvalidDomain)
        )))
    ));
    let mut raw = original.clone();
    raw["selection"]["polynomial_sources"][0]["construction"]["max_axis_degree"] = 1.into();
    assert!(matches!(
        parsed(&raw),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PolynomialTopology(BivariateTopologyError::DegreeLimit)
        )))
    ));
    let mut raw = original.clone();
    raw["selection"]["polynomial_sources"][0]["events"] = json!([]);
    assert!(matches!(parsed(&raw), Err(PersistenceError::Json(_))));
    let mut raw = original.clone();
    raw["version"] = 12.into();
    assert_eq!(
        parsed(&raw).unwrap_err(),
        PersistenceError::UnsupportedVersion(12)
    );
    let mut raw = original.clone();
    raw["selection"]
        .as_object_mut()
        .unwrap()
        .remove("polynomial_sources");
    assert!(matches!(parsed(&raw), Err(PersistenceError::Json(_))));
    for (path, value, limits, resource) in [
        (
            "/selection/polynomial_sources/0/construction/max_projection_degree",
            json!(257),
            ArchiveLimits::default(),
            ArchiveResource::Degree,
        ),
        (
            "/selection/polynomial_sources/0/evaluation/refinement_steps",
            json!(1025),
            ArchiveLimits::default(),
            ArchiveResource::SolverSteps,
        ),
        (
            "/selection/polynomial_sources/0/construction/max_refinement_rounds",
            json!(1025),
            ArchiveLimits::default(),
            ArchiveResource::SolverSteps,
        ),
        (
            "/selection/polynomial_sources/0/construction/fiber/max_subdivision_depth",
            json!(1025),
            ArchiveLimits::default(),
            ArchiveResource::SolverSteps,
        ),
        (
            "/selection/polynomial_sources/0/construction/max_axis_degree",
            json!(8),
            ArchiveLimits {
                degree: 1,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Degree,
        ),
        (
            "/selection/polynomial_sources/0/construction/max_axis_degree",
            json!(8),
            ArchiveLimits {
                geometry_elements: 0,
                ..ArchiveLimits::default()
            },
            ArchiveResource::GeometryElements,
        ),
    ] {
        let mut raw = original.clone();
        *raw.pointer_mut(path).unwrap() = value;
        raw["selection"]["polynomial_sources"][0]["equation"][0][0] = json!("bad scalar");
        assert!(
            matches!(RawModel::from_json(&raw.to_string(),&limits),Err(PersistenceError::ResourceLimit(error)) if error.resource==resource)
        );
    }
    parsed(&original).unwrap();
}

pub fn branch_replay_policy_budget_and_recovery() {
    let limited = PolynomialCurveSet2::from_topology(
        circle().topology().clone(),
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 0,
            refinement_steps: 0,
        },
    )
    .unwrap();
    let restored = replay(&model(limited.curves()), "limited-evaluation");
    for curve in restored.pcurves() {
        let curve = curve.geometry().curve();
        assert_eq!(
            curve
                .polynomial_branch()
                .unwrap()
                .evaluation_config()
                .max_subdivision_depth,
            0
        );
        assert!(
            matches!(curve.point_at(&q(3,8).into()), Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason()==hypercurve::UncertaintyReason::BudgetExhausted)
        );
    }
    let ordinary = circle();
    let mixed = model(&[limited.curves()[0].clone(), ordinary.curves()[0].clone()]);
    let mixed = replay(&mixed, "distinct-controls");
    let branches = mixed
        .pcurves()
        .map(|p| p.geometry().curve().polynomial_branch().unwrap())
        .collect::<Vec<_>>();
    assert!(!branches[0].shares_source(branches[1]));
    assert_eq!(branches[0].evaluation_config().max_subdivision_depth, 0);
    assert_eq!(branches[1].evaluation_config().max_subdivision_depth, 93);
    let original = model(circle().curves());
    let archive = original.to_json().unwrap();
    let exact = ArchiveLimits {
        solver_steps: 128,
        ..ArchiveLimits::default()
    };
    RawModel::from_json(&archive, &exact)
        .unwrap()
        .validate()
        .unwrap();
    assert!(
        matches!(RawModel::from_json(&archive, &ArchiveLimits {solver_steps:127, ..exact}),Err(PersistenceError::ResourceLimit(error)) if error.resource == ArchiveResource::SolverSteps && error.observed == 128)
    );
    let raw = RawModel::from_json(&archive, &ArchiveLimits::default()).unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| raw.validate());
    assert_eq!(denied.result.unwrap_err().limit, 0);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let restored = replay(&original, "policy");
            inspect_sources(&restored);
            for curve in restored.pcurves() {
                let point = curve.geometry().curve().point_at(&q(1, 2).into()).unwrap();
                assert_eq!(
                    point
                        .affine_form(1.into(), 0.into(), 0.into())
                        .compare_to_real(&q(1, 2))
                        .unwrap(),
                    Ordering::Equal
                );
            }
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
}
