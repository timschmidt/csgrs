use hyperbrep::*;
use hypercurve::{Curve2, RationalMap2};
use hypersolve::BivariatePolynomial;
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn polynomial(rows: Vec<Vec<Real>>) -> BivariatePolynomial {
    BivariatePolynomial::new(rows)
}
fn one() -> BivariatePolynomial {
    polynomial(vec![vec![1.into()]])
}
fn x() -> BivariatePolynomial {
    polynomial(vec![vec![0.into()], vec![1.into()]])
}
fn y() -> BivariatePolynomial {
    polynomial(vec![vec![0.into(), 1.into()]])
}
fn circle() -> BivariatePolynomial {
    polynomial(vec![
        vec![q(7, 16), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ])
}
fn binomial(n: usize, k: usize) -> i32 {
    (0..k).fold(1, |value, i| value * (n - i) as i32 / (i + 1) as i32)
}
// Independent power-to-Bernstein identity supplies exact authored test nets.
fn tensor(coordinates: [BivariatePolynomial; 3], weight: BivariatePolynomial) -> Surface {
    let polynomials = [
        coordinates[0].clone(),
        coordinates[1].clone(),
        coordinates[2].clone(),
        weight,
    ];
    let n = polynomials
        .iter()
        .map(|p| p.coefficients.len().saturating_sub(1))
        .max()
        .unwrap()
        .max(2);
    let m = polynomials
        .iter()
        .flat_map(|p| &p.coefficients)
        .map(|row| row.len().saturating_sub(1))
        .max()
        .unwrap()
        .max(2);
    assert!(n <= 8 && m <= 8);
    let bernstein = |p: &BivariatePolynomial, i, j| {
        let mut value = Real::zero();
        for (k, row) in p.coefficients.iter().enumerate().take(i + 1) {
            for (l, coefficient) in row.iter().enumerate().take(j + 1) {
                value += coefficient
                    * q(binomial(i, k), binomial(n, k))
                    * q(binomial(j, l), binomial(m, l));
            }
        }
        value
    };
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for j in 0..=m {
        let mut row = Vec::new();
        let mut wrow = Vec::new();
        for i in 0..=n {
            let w = bernstein(&polynomials[3], i, j);
            let coords = (0..3)
                .map(|axis| (bernstein(&polynomials[axis], i, j) / &w).unwrap())
                .collect::<Vec<_>>();
            row.push(Point3::new(
                coords[0].clone(),
                coords[1].clone(),
                coords[2].clone(),
            ));
            wrow.push(w);
        }
        points.push(row);
        weights.push(wrow);
    }
    Surface::rational_bezier(points, weights).unwrap()
}
fn plane() -> Surface {
    Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap()
}
fn branches(result: SurfaceSurfaceIntersection) -> Vec<SurfaceIntersectionCurve> {
    match result {
        SurfaceSurfaceIntersection::Curve(c) => vec![*c],
        SurfaceSurfaceIntersection::Curves(c) => c,
        other => panic!("expected complete curve branches, got {other:?}"),
    }
}
fn check_correspondence(section: &SurfaceIntersectionCurve, first: &Surface, second: &Surface) {
    let source = section.second_pcurve().materialize().unwrap();
    assert!(source.curve().polynomial_branch().is_some());
    let samples = [
        section.curve().domain().start().clone(),
        section.curve().domain().end().clone(),
    ];
    for parameter in samples {
        let spatial = section.curve().point_at(&parameter).unwrap();
        let a = first
            .point_at(&section.first_pcurve().point_at(&parameter).unwrap())
            .unwrap();
        let b = second
            .point_at(&section.second_pcurve().point_at(&parameter).unwrap())
            .unwrap();
        assert!(spatial.coincides_with(&a).unwrap());
        assert!(spatial.coincides_with(&b).unwrap());
        if let Some((u, v)) = first.plane_directions() {
            let normal = u.cross(v);
            let origin = first.plane_origin().unwrap();
            let rhs = &normal[0] * &origin.x + &normal[1] * &origin.y + &normal[2] * &origin.z;
            assert_eq!(
                spatial.compare_linear_form(&normal, &rhs).unwrap(),
                Ordering::Equal
            );
        }
        spatial.bounds().unwrap();
    }
}
fn artifact(tag: &str, archive: &str) {
    if let Some(directory) = std::env::var_os("HYPERBREP_PLANE_TENSOR_ARTIFACT_DIR") {
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(
            std::path::Path::new(&directory).join(format!("{tag}.json")),
            archive,
        )
        .unwrap();
    }
}

fn revalidate_recorded_images(model: &Model, correspondences: &[(Real, Real)], label: &str) {
    let mut builder = ModelBuilder::new();
    let surfaces = model
        .surfaces()
        .map(|surface| builder.surface(surface.geometry().clone()).unwrap())
        .collect::<Vec<_>>();
    let pcurves = model
        .pcurves()
        .map(|pcurve| builder.pcurve(pcurve.geometry().clone()).unwrap())
        .collect::<Vec<_>>();
    for (index, curve) in model.curves().enumerate() {
        let curve = curve.geometry();
        let start = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let end = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let domain = curve.domain().clone();
        let curve = builder.curve(curve.clone()).unwrap();
        let edge = builder.edge(&start, &end, &curve, domain).unwrap();
        for operand in 0..2 {
            let (scale, offset) = &correspondences[2 * index + operand];
            builder
                .validate_edge_image(
                    &edge,
                    Direction::Forward,
                    &pcurves[2 * index + operand],
                    ParameterCorrespondence::affine(scale.clone(), offset.clone()).unwrap(),
                    &surfaces[1 - operand],
                )
                .unwrap_or_else(|error| {
                    panic!("{label}: branch {index}, operand {operand}: {error:?}")
                });
        }
    }
    builder.finish().unwrap();
}

fn model_and_replay(sections: &[SurfaceIntersectionCurve], surface: &Surface, tag: &str) -> Model {
    let mut builder = ModelBuilder::new();
    let mut correspondences = Vec::new();
    let support = builder.surface(surface.clone()).unwrap();
    let cap = builder.surface(plane()).unwrap();
    for section in sections {
        let curve = section.curve();
        let a = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let b = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let domain = curve.domain().clone();
        let source = builder.curve(curve.clone()).unwrap();
        let edge = builder.edge(&a, &b, &source, domain).unwrap();
        for (operand, (pcurve, surface)) in [
            (section.first_pcurve(), &cap),
            (section.second_pcurve(), &support),
        ]
        .into_iter()
        .enumerate()
        {
            let materialized = pcurve.materialize().unwrap();
            let (scale, offset) = materialized.correspondence().affine_coefficients().unwrap();
            correspondences.push((scale.clone(), offset.clone()));
            let pcurve = builder
                .pcurve(Pcurve::new(materialized.curve().clone()))
                .unwrap();
            builder
                .validate_edge_image(
                    &edge,
                    Direction::Forward,
                    &pcurve,
                    ParameterCorrespondence::affine(scale.clone(), offset.clone()).unwrap(),
                    surface,
                )
                .unwrap_or_else(|error| panic!("{tag} operand {operand}: {error:?}"));
        }
    }
    let model = builder.finish().unwrap();
    let archive = model.to_json().unwrap();
    artifact(tag, &archive);
    let restored = RawModel::from_json(&archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), archive);
    for (a, b) in model.curves().zip(restored.curves()) {
        let (a, b) = (a.geometry(), b.geometry());
        for t in [a.domain().start(), a.domain().end()] {
            assert!(
                a.point_at(t)
                    .unwrap()
                    .coincides_with(&b.point_at(t).unwrap())
                    .unwrap()
            );
        }
    }
    revalidate_recorded_images(&restored, &correspondences, &format!("{tag}-replay"));
    let transform = Matrix4::from_row_major([
        1.into(),
        1.into(),
        0.into(),
        2.into(),
        0.into(),
        2.into(),
        0.into(),
        (-1).into(),
        1.into(),
        0.into(),
        1.into(),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let transformed = restored.transformed(&transform).unwrap();
    let archive = transformed.to_json().unwrap();
    artifact(&format!("{tag}-affine"), &archive);
    let transformed = RawModel::from_json(&archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(transformed.to_json().unwrap(), archive);
    for (source, image) in restored.curves().zip(transformed.curves()) {
        for t in [
            source.geometry().domain().start(),
            source.geometry().domain().end(),
        ] {
            let expected = source
                .geometry()
                .point_at(t)
                .unwrap()
                .transformed(&transform)
                .unwrap();
            assert!(
                expected
                    .coincides_with(&image.geometry().point_at(t).unwrap())
                    .unwrap()
            );
        }
    }
    revalidate_recorded_images(
        &transformed,
        &correspondences,
        &format!("{tag}-affine-replay"),
    );
    restored
}

pub fn original_full_interior_loop() {
    // z=(u-1/2)^2+(v-1/2)^2-1/16. Its zero set is a complete
    // radius-1/4 circle strictly inside [0,1]^2, with no boundary seed.
    let a = [Real::zero(), q(-1, 2), Real::zero()];
    let controls = (0..3)
        .map(|v| {
            (0..3)
                .map(|u| Point3::new(q(u as i32, 2), q(v as i32, 2), q(7, 16) + &a[u] + &a[v]))
                .collect()
        })
        .collect();
    let surface = Surface::rational_bezier(controls, vec![vec![Real::one(); 3]; 3]).unwrap();
    let plane = Surface::plane(
        Point3::new(0.into(), 0.into(), 0.into()),
        Vector3::new([1.into(), 0.into(), 0.into()]),
        Vector3::new([0.into(), 1.into(), 0.into()]),
    )
    .unwrap();
    for (u, v) in [
        (q(1, 4), q(1, 2)),
        (q(3, 4), q(1, 2)),
        (q(1, 2), q(1, 4)),
        (q(1, 2), q(3, 4)),
    ] {
        let point = surface
            .point_at(&hypercurve::Point2::new(u, v).into())
            .unwrap();
        assert!(
            point
                .try_coordinates()
                .expect("represented original hard case")
                .z
                .exact_rational_normal_form()
                .unwrap()
                .is_zero()
        );
    }
    let section = plane.intersect_surface(&surface);
    println!("complete interior circle plane section: {section:?}");
    assert!(
        section.is_ok(),
        "an interior-loop section needs a complete surface intersection result"
    );
}

pub fn full_loops_and_nonlinear_weighted_images() {
    for weighted in [false, true] {
        let nx = if weighted {
            polynomial(vec![
                vec![0.into(), 0.into(), q(1, 32)],
                vec![1.into()],
                vec![q(1, 16)],
            ])
        } else {
            x()
        };
        let ny = if weighted {
            polynomial(vec![vec![0.into(), 1.into()], vec![0.into(), q(1, 32)]])
        } else {
            y()
        };
        let weight = if weighted {
            polynomial(vec![vec![1.into()], vec![q(1, 8)]])
        } else {
            one()
        };
        let surface = tensor([nx, ny, circle()], weight);
        let sections = branches(plane().intersect_surface(&surface).unwrap());
        assert_eq!(
            sections.len(),
            2,
            "both halves of the complete interior loop"
        );
        for (branch, section) in sections.iter().enumerate() {
            check_correspondence(section, &plane(), &surface);
            let reversed = section.reversed().unwrap();
            check_correspondence(&reversed, &plane(), &surface);
            model_and_replay(
                &[reversed],
                &surface,
                &format!(
                    "{}-reversed-{branch}",
                    if weighted { "weighted" } else { "plain" }
                ),
            );
            let range = ParameterRange::new(
                section.curve().domain().start().clone(),
                Parameter::from_real(q(1, 2)),
            )
            .unwrap();
            let restricted = section.restrict_source(range).unwrap();
            check_correspondence(&restricted, &plane(), &surface);
            model_and_replay(
                &[restricted],
                &surface,
                &format!(
                    "{}-restricted-{branch}",
                    if weighted { "weighted" } else { "plain" }
                ),
            );
            let middle = section
                .curve()
                .point_at(&Parameter::from_real(q(1, 2)))
                .unwrap();
            let uv = section
                .second_pcurve()
                .point_at(&Parameter::from_real(q(1, 2)))
                .unwrap();
            assert_eq!(
                uv.affine_form(1.into(), 0.into(), 0.into())
                    .compare_to_real(&q(1, 2))
                    .unwrap(),
                Ordering::Equal
            );
            assert!(
                middle
                    .coincides_with(&surface.point_at(&uv).unwrap())
                    .unwrap()
            );
        }
        model_and_replay(
            &sections,
            &surface,
            if weighted {
                "nonlinear-weighted-loop"
            } else {
                "interior-loop"
            },
        );
    }
}

pub fn noncircular_singular_and_isolated_sections() {
    let equations = [
        polynomial(vec![
            vec![q(-1, 8), 0.into(), 0.into(), 1.into()],
            vec![q(-1, 8)],
        ]),
        polynomial(vec![
            vec![q(3, 8), (-1).into(), 1.into()],
            vec![q(-3, 4)],
            vec![q(3, 2)],
            vec![(-1).into()],
        ]),
        polynomial(vec![
            vec![0.into(), (-1).into(), 1.into()],
            vec![1.into()],
            vec![(-1).into()],
        ]),
    ];
    for (index, equation) in equations.into_iter().enumerate() {
        let surface = tensor([x(), y(), equation], one());
        let sections = branches(plane().intersect_surface(&surface).unwrap());
        assert_eq!(
            sections.len(),
            [1, 2, 4][index],
            "complete cubic, cusp or crossing-node graph"
        );
        for section in &sections {
            check_correspondence(section, &plane(), &surface);
        }
        model_and_replay(&sections, &surface, &format!("noncircular-{index}"));
    }
    let point_equation = polynomial(vec![
        vec![q(1, 2), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let surface = tensor([x(), y(), point_equation], one());
    let SurfaceSurfaceIntersection::Point(point) = plane().intersect_surface(&surface).unwrap()
    else {
        panic!("complete isolated contact");
    };
    assert!(
        point
            .coincides_with(&Point3::new(q(1, 2), q(1, 2), 0.into()).into())
            .unwrap()
    );
    let algebraic = polynomial(vec![
        vec![q(1, 2), (-1).into(), 1.into()],
        vec![],
        vec![(-1).into()],
        vec![],
        vec![1.into()],
    ]);
    let surface = tensor([x(), y(), algebraic], one());
    let SurfaceSurfaceIntersection::Point(point) = plane().intersect_surface(&surface).unwrap()
    else {
        panic!("complete nonrational isolated contact");
    };
    assert!(
        point.try_coordinates().is_none(),
        "the selected source must remain authoritative"
    );
    let expected = Point3::new(q(1, 2).sqrt().unwrap(), q(1, 2), 0.into());
    assert!(point.coincides_with(&expected.into()).unwrap());
    let mut builder = ModelBuilder::new();
    builder.vertex(*point).unwrap();
    let archive = builder.finish().unwrap().to_json().unwrap();
    artifact("isolated-selected", &archive);
    let model = RawModel::from_json(&archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), archive);
}

pub fn rational_image_poles_and_replay_guards() {
    let surface = tensor([x(), y(), circle()], one());
    let sections = branches(plane().intersect_surface(&surface).unwrap());
    let materialized = sections[0].second_pcurve().materialize().unwrap();
    let source: &Curve2 = materialized.curve();
    let denominator = polynomial(vec![vec![q(-1, 2)], vec![1.into()]]);
    let map = RationalMap2::new([denominator.clone(), denominator.clone()], denominator);
    assert!(
        source.rational_image(&map).is_err(),
        "a cancellable pole still invalidates a whole image"
    );
    assert!(
        source
            .point_at(&q(1, 2).into())
            .unwrap()
            .rational_image(&map)
            .is_err()
    );
    let model = model_and_replay(&sections, &surface, "guard-source");
    let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    raw["pcurves"][0]["PolynomialBranch"]["maps"][0]["denominator"] =
        serde_json::to_value(vec![vec![Real::zero()]]).unwrap();
    assert!(matches!(
        RawModel::from_json(
            &serde_json::to_string(&raw).unwrap(),
            &ArchiveLimits::default()
        )
        .and_then(RawModel::validate),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PlanarCurve(hypercurve::ExactCurveError::Invalid {
                cause: hypercurve::CurveError::ZeroRationalPointDenominator,
                ..
            })
        )))
    ));
    let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    raw["pcurves"][0]["PolynomialBranch"]["maps"][0]["numerators"][0] = serde_json::json!(
        (0..258)
            .map(|_| vec!["malformed scalar"])
            .collect::<Vec<_>>()
    );
    assert!(
        matches!(RawModel::from_json(&serde_json::to_string(&raw).unwrap(),&ArchiveLimits::default()),Err(PersistenceError::ResourceLimit(limit)) if limit.resource==ArchiveResource::Degree && limit.observed==257)
    );
    let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    raw["version"] = 13.into();
    assert!(matches!(
        RawModel::from_json(
            &serde_json::to_string(&raw).unwrap(),
            &ArchiveLimits::default()
        ),
        Err(PersistenceError::UnsupportedVersion(13))
    ));
}

pub fn image_refusal_budget_and_recovery() {
    let folded = polynomial(vec![vec![q(1, 4)], vec![(-1).into()], vec![1.into()]]);
    let surface = tensor([folded, y(), circle()], one());
    assert!(
        plane().intersect_surface(&surface).is_err(),
        "folded projection needs its own image events"
    );
    let surface = tensor([x(), y(), circle()], one());
    let denied =
        OperationContext::strict(WorkBudget::ZERO).evaluate(|| plane().intersect_surface(&surface));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let sections = branches(plane().intersect_surface(&surface).unwrap());
            assert_eq!(sections.len(), 2);
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }

    // Agreement at both projection-event endpoints is insufficient: this
    // surface adds (u-1/4)(u-3/4) to z, which is nonzero inside each branch.
    let bumped = tensor(
        [
            x(),
            y(),
            hypersolve::bivariate_arithmetic::bivariate_add(
                &circle(),
                &polynomial(vec![vec![q(3, 16)], vec![(-1).into()], vec![1.into()]]),
            ),
        ],
        one(),
    );
    for section in branches(plane().intersect_surface(&surface).unwrap()) {
        for parameter in [
            section.curve().domain().start(),
            section.curve().domain().end(),
        ] {
            let uv = section.second_pcurve().point_at(parameter).unwrap();
            assert!(
                section
                    .curve()
                    .point_at(parameter)
                    .unwrap()
                    .coincides_with(&bumped.point_at(&uv).unwrap())
                    .unwrap()
            );
        }
        let middle = Parameter::from_real(q(1, 2));
        let uv = section.second_pcurve().point_at(&middle).unwrap();
        assert!(
            !section
                .curve()
                .point_at(&middle)
                .unwrap()
                .coincides_with(&bumped.point_at(&uv).unwrap())
                .unwrap()
        );
        let mut builder = ModelBuilder::new();
        let curve = section.curve();
        let start = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let end = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let source = builder.curve(curve.clone()).unwrap();
        let edge = builder
            .edge(&start, &end, &source, curve.domain().clone())
            .unwrap();
        let materialized = section.second_pcurve().materialize().unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(materialized.curve().clone()))
            .unwrap();
        let support = builder.surface(bumped.clone()).unwrap();
        let (scale, offset) = materialized.correspondence().affine_coefficients().unwrap();
        assert!(matches!(
            builder.validate_edge_image(
                &edge,
                Direction::Forward,
                &pcurve,
                ParameterCorrespondence::affine(scale.clone(), offset.clone()).unwrap(),
                &support,
            ),
            Err(BuildError::EdgeUseSupportMismatch
                | BuildError::Geometry(GeometryError::UnrepresentableParameter))
        ));
    }
}

pub fn disconnected_mixed_and_near_coincident_sections() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    let norm = polynomial(vec![
        vec![q(1, 2), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let mixed = tensor([x(), y(), bivariate_multiply(&circle(), &norm)], one());
    let SurfaceSurfaceIntersection::Components(components) =
        plane().intersect_surface(&mixed).unwrap()
    else {
        panic!("isolated center and complete circle");
    };
    assert_eq!(components.points().len(), 1);
    assert!(
        components.points()[0]
            .coincides_with(&Point3::new(q(1, 2), q(1, 2), 0.into()).into())
            .unwrap()
    );
    assert_eq!(components.surface_curves().len(), 4);
    assert!(components.curves().is_empty());
    for section in components.surface_curves() {
        check_correspondence(section, &plane(), &mixed);
    }
    model_and_replay(components.surface_curves(), &mixed, "mixed-circle-point");
    let small = polynomial(vec![
        vec![q(31, 64), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let disconnected = tensor([x(), y(), bivariate_multiply(&circle(), &small)], one());
    let sections = branches(plane().intersect_surface(&disconnected).unwrap());
    assert_eq!(
        sections.len(),
        8,
        "both complete concentric loops split at all projection events"
    );
    model_and_replay(&sections, &disconnected, "disconnected-loops");
    let epsilon = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    let tiny = polynomial(vec![
        vec![q(1, 2) - &epsilon * &epsilon, (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let surface = tensor([x(), y(), tiny], one());
    let sections = branches(plane().intersect_surface(&surface).unwrap());
    assert_eq!(sections.len(), 2);
    let points = sections
        .iter()
        .map(|section| {
            section
                .curve()
                .point_at(&Parameter::from_real(q(1, 2)))
                .unwrap()
        })
        .collect::<Vec<_>>();
    assert!(!points[0].coincides_with(&points[1]).unwrap());
    for (point, expected) in points.iter().zip([q(1, 2) - &epsilon, q(1, 2) + &epsilon]) {
        assert_eq!(
            point.compare_linear_form(&Vector3::y(), &expected).unwrap(),
            Ordering::Equal
        );
    }
    model_and_replay(&sections, &surface, "tiny-interior-loop");
}

pub fn empty_contained_boundary_and_solver_frontier() {
    use hypersolve::bivariate_topology::BivariateTopologyError;
    for value in [-1, 1] {
        let surface = tensor([x(), y(), polynomial(vec![vec![value.into()]])], one());
        assert!(matches!(
            plane().intersect_surface(&surface).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
    }
    let surface = tensor([x(), y(), polynomial(Vec::new())], one());
    assert!(matches!(
        plane().intersect_surface(&surface).unwrap(),
        SurfaceSurfaceIntersection::ContainedSurface(SurfaceIntersectionOperand::Second)
    ));
    assert!(matches!(
        surface.intersect_surface(&plane()).unwrap(),
        SurfaceSurfaceIntersection::ContainedSurface(SurfaceIntersectionOperand::First)
    ));
    let surface = tensor(
        [
            x(),
            y(),
            polynomial(vec![
                vec![0.into(), 0.into(), 1.into()],
                vec![],
                vec![1.into()],
            ]),
        ],
        one(),
    );
    let SurfaceSurfaceIntersection::Point(point) = plane().intersect_surface(&surface).unwrap()
    else {
        panic!("boundary tangency is an isolated point");
    };
    assert!(point.coincides_with(&Point3::origin().into()).unwrap());
    // Preserve the original z=v boundary-section fixture as exact success,
    // together with its upper-boundary counterpart z=v-1.
    for (equation, ordinate, tag) in [
        (y(), Real::zero(), "lower-boundary-section"),
        (
            polynomial(vec![vec![(-1).into(), 1.into()]]),
            Real::one(),
            "upper-boundary-section",
        ),
    ] {
        let surface = tensor([x(), y(), equation], one());
        let sections = branches(plane().intersect_surface(&surface).unwrap());
        assert_eq!(sections.len(), 1);
        let section = &sections[0];
        check_correspondence(section, &plane(), &surface);
        let restricted = section
            .subcurve(&q(1, 4), &q(3, 4))
            .unwrap()
            .reversed()
            .unwrap();
        check_correspondence(&restricted, &plane(), &surface);
        let restored = model_and_replay(&[restricted], &surface, tag);
        let cutting = Surface::plane(
            Point3::new(q(1, 2), 0.into(), 0.into()),
            Vector3::y(),
            Vector3::z(),
        )
        .unwrap();
        for curve in restored.curves() {
            let CurveSurfaceIntersection::Points(contacts) =
                cutting.intersect_curve(curve.geometry()).unwrap()
            else {
                panic!("the replayed boundary section has one transverse contact");
            };
            assert_eq!(contacts.len(), 1);
            assert!(
                contacts[0]
                    .point
                    .coincides_with(&Point3::new(q(1, 2), ordinate.clone(), 0.into()).into())
                    .unwrap()
            );
            assert!(
                contacts[0]
                    .point
                    .coincides_with(&curve.geometry().point_at(&contacts[0].parameter).unwrap())
                    .unwrap()
            );
        }
    }
    for (equation, expected) in [
        (
            polynomial(vec![vec![q(-1, 2)], vec![1.into()]]),
            BivariateTopologyError::VerticalComponent,
        ),
        (
            polynomial(vec![vec![q(1, 4), (-1).into(), 1.into()]]),
            BivariateTopologyError::RepeatedComponent,
        ),
    ] {
        let surface = tensor([x(), y(), equation], one());
        assert!(
            matches!(plane().intersect_surface(&surface),Err(GeometryError::PolynomialTopology(error)) if error==expected)
        );
    }
}
