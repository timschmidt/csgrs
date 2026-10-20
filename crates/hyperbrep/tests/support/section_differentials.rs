use hyperbrep::*;
use hypercurve::{CurveParameter2, CurveVector2, RationalMap2};
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

fn vector_power(vector: &CurveVector2, power: usize, rhs: &Real) {
    let mut row = vec![Real::zero(); power + 1];
    row[power] = Real::one();
    assert_eq!(
        vector
            .rational_form(polynomial(vec![row]), one())
            .unwrap()
            .compare_to_real(rhs)
            .unwrap(),
        Ordering::Equal
    );
}
fn coordinate(vector: &GeometricVector3, axis: Axis, value: &Real) {
    assert_eq!(
        vector.compare_coordinate(axis, value).unwrap(),
        Ordering::Equal
    );
}
fn shear() -> Matrix4 {
    Matrix4::from_row_major([
        2.into(),
        3.into(),
        0.into(),
        7.into(),
        0.into(),
        (-1).into(),
        0.into(),
        (-4).into(),
        1.into(),
        0.into(),
        4.into(),
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}
fn archive_curve(curve: &Curve3, tag: &str) -> Curve3 {
    let mut builder = ModelBuilder::new();
    let a = builder
        .vertex(curve.point_at(curve.domain().start()).unwrap())
        .unwrap();
    let b = builder
        .vertex(curve.point_at(curve.domain().end()).unwrap())
        .unwrap();
    let c = builder.curve(curve.clone()).unwrap();
    builder.edge(&a, &b, &c, curve.domain().clone()).unwrap();
    let model = builder.finish().unwrap();
    let archive = model.to_json().unwrap();
    if let Some(dir) = std::env::var_os("HYPERBREP_SECTION_DIFFERENTIAL_ARTIFACT_DIR") {
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            std::path::Path::new(&dir).join(format!("{tag}.json")),
            &archive,
        )
        .unwrap();
    }
    let restored = RawModel::from_json(&archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), archive);
    restored.curves().next().unwrap().geometry().clone()
}
fn reversed_parameter(curve: &Curve3, reversed: &Curve3, parameter: &Parameter) -> Parameter {
    let a = curve.planar_geometry().unwrap();
    let b = reversed.planar_geometry().unwrap();
    let scale = (a.source_scale / b.source_scale).unwrap();
    let offset = ((a.source_offset - b.source_offset) / b.source_scale).unwrap();
    parameter.affine_image(&scale, &offset).unwrap()
}
fn check_circle(curve: &Curve3, scale: i32) {
    let frame = curve.planar_geometry().unwrap();
    let parameter =
        Parameter::from_real(((q(3, 5) - frame.source_offset) / frame.source_scale).unwrap());
    for order in 1..=3 {
        let vector = curve.derivative_at(&parameter, order).unwrap();
        assert!(
            vector.try_coordinates().is_none(),
            "joint selected fiber is retained"
        );
        coordinate(&vector, Axis::X, &Real::from(scale * i32::from(order == 1)));
        coordinate(&vector, Axis::Z, &Real::zero());
        assert!(!vector.is_zero().unwrap());
    }
}

pub fn regular_sections_restriction_transform_and_replay() {
    let surface = tensor([x(), y(), circle()], one());
    let sections = branches(plane().intersect_surface(&surface).unwrap());
    assert_eq!(sections.len(), 2);
    for (index, section) in sections.iter().enumerate() {
        check_circle(section.curve(), 1);
        for pcurve in [section.first_pcurve(), section.second_pcurve()] {
            let pcurve = pcurve.materialize().unwrap();
            assert_eq!(
                pcurve.correspondence().affine_coefficients().unwrap(),
                (&Real::one(), &Real::zero())
            );
            let jets = pcurve.curve().derivatives_at(&q(3, 5).into(), 3).unwrap();
            assert_eq!(jets.len(), 3);
            vector_power(&jets[0], 2, &q(4, 21));
            vector_power(&jets[1], 2, &q(250000, 9261));
            // Third derivative: -3 r^2 (x-1/2)/(y-1/2)^5.
            vector_power(&jets[2], 2, &q(400000000, 453789));
            for jet in &jets {
                let point = jet.coordinate_image().unwrap();
                let replay = point.definition().unwrap().into_point().unwrap();
                assert!(point.coincides_with(&replay).unwrap());
            }
        }
        let reverse = section.reversed().unwrap();
        check_circle(reverse.curve(), -1);
        let restricted = section
            .restrict_source(ParameterRange::new(q(3, 5).into(), q(2, 3).into()).unwrap())
            .unwrap();
        check_circle(restricted.curve(), 1);
        for (name, curve) in [
            ("section", section.curve()),
            ("reversed", reverse.curve()),
            ("restricted", restricted.curve()),
        ] {
            check_circle(
                &archive_curve(curve, &format!("circle-{index}-{name}")),
                if name == "reversed" { -1 } else { 1 },
            );
        }
        let image = Curve3::affine_image(section.curve().clone(), shear()).unwrap();
        for curve in [
            image.clone(),
            archive_curve(&image, &format!("circle-{index}-affine")),
        ] {
            for order in 1..=3 {
                let p = q(3, 5).into();
                let v = curve.derivative_at(&p, order).unwrap();
                let transported = section
                    .curve()
                    .derivative_at(&p, order)
                    .unwrap()
                    .affine_image(&shear())
                    .unwrap();
                let direction = Vector3::new([1.into(), 3.into(), 0.into()]);
                let expected = if order == 1 { 2.into() } else { 0.into() };
                for value in [v, transported] {
                    assert_eq!(
                        value.compare_linear_form(&direction, &expected).unwrap(),
                        Ordering::Equal
                    );
                    coordinate(&value, Axis::Z, &Real::from(i32::from(order == 1)));
                    let epsilon = q(1, 1_000_000_000) * q(1, 1000);
                    assert_eq!(
                        value
                            .compare_linear_form(&direction, &(&expected + &epsilon))
                            .unwrap(),
                        Ordering::Less
                    );
                    assert_eq!(
                        value
                            .compare_linear_form(&direction, &(&expected - &epsilon))
                            .unwrap(),
                        Ordering::Greater
                    );
                }
            }
        }
    }
}

fn selected_parameter() -> Parameter {
    let curve = Curve3::rational_bezier(
        [-1, -1, -1, 4]
            .map(|n| Point3::new(n.into(), 0.into(), 0.into()))
            .to_vec(),
        vec![Real::one(); 4],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("one selected cubic root")
    };
    assert_eq!(parameters.len(), 1);
    let p = parameters.pop().unwrap();
    assert!(p.try_real().is_none());
    p
}

pub fn selected_parameters_and_native_spatial_vectors() {
    let parameter = selected_parameter();
    let controls = [-1, -1, -1, 4]
        .map(|n| Point3::new(n.into(), (2 * n).into(), (3 * n).into()))
        .to_vec();
    let curves = [
        Curve3::rational_bezier(controls.clone(), vec![Real::one(); 4]).unwrap(),
        Curve3::nurbs(
            3,
            controls.clone(),
            vec![Real::one(); 4],
            vec![
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
                1.into(),
                1.into(),
                1.into(),
            ],
        )
        .unwrap(),
        Curve3::nurbs(
            3,
            controls,
            vec![Real::one(); 4],
            vec![
                2.into(),
                2.into(),
                2.into(),
                2.into(),
                5.into(),
                5.into(),
                5.into(),
                5.into(),
            ],
        )
        .unwrap(),
    ];
    for (index, curve) in curves.iter().enumerate() {
        let parameter = if index == 2 {
            parameter.affine_image(&3.into(), &2.into()).unwrap()
        } else {
            parameter.clone()
        };
        for source in [
            curve.clone(),
            archive_curve(curve, &format!("native-selected-{index}")),
        ] {
            for order in 1..=4 {
                let vector = source.derivative_at(&parameter, order).unwrap();
                assert!(vector.try_coordinates().is_none());
                for direction in [
                    Vector3::new([2.into(), (-1).into(), 0.into()]),
                    Vector3::new([3.into(), 0.into(), (-1).into()]),
                ] {
                    assert_eq!(
                        vector
                            .compare_linear_form(&direction, &Real::zero())
                            .unwrap(),
                        Ordering::Equal
                    );
                }
                assert_eq!(vector.is_zero().unwrap(), order == 4);
                if order == 3 {
                    coordinate(
                        &vector,
                        Axis::X,
                        &if index == 2 { q(10, 9) } else { 30.into() },
                    );
                }
                let transformed = vector.affine_image(&shear()).unwrap();
                // M(1,2,3)=(8,-2,13); orthogonal linear forms are exact.
                assert_eq!(
                    transformed
                        .compare_linear_form(
                            &Vector3::new([1.into(), 4.into(), 0.into()]),
                            &0.into()
                        )
                        .unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    transformed
                        .compare_linear_form(
                            &Vector3::new([13.into(), 0.into(), (-8).into()]),
                            &0.into()
                        )
                        .unwrap(),
                    Ordering::Equal
                );
                let image = Curve3::affine_image(source.clone(), shear()).unwrap();
                assert_eq!(
                    image
                        .derivative_at(&parameter, order)
                        .unwrap()
                        .compare_linear_form(
                            &Vector3::new([1.into(), 4.into(), 0.into()]),
                            &0.into()
                        )
                        .unwrap(),
                    Ordering::Equal
                );
            }
            assert_eq!(
                source.derivative_at(&parameter, usize::MAX).unwrap_err(),
                GeometryError::Numeric(hyperreal::Problem::OutOfRange)
            );
        }
    }
    let sections = branches(
        plane()
            .intersect_surface(&tensor([x(), y(), circle()], one()))
            .unwrap(),
    );
    for (index, section) in sections.iter().enumerate() {
        let vector = section.curve().derivative_at(&parameter, 1).unwrap();
        let reversed = section.reversed().unwrap();
        let reflected = reversed_parameter(section.curve(), reversed.curve(), &parameter);
        assert!(reflected.try_real().is_none());
        let reverse_vector = reversed.curve().derivative_at(&reflected, 1).unwrap();
        coordinate(&reverse_vector, Axis::X, &(-Real::one()));
        assert_eq!(
            reverse_vector.coordinate_sign(Axis::Y).unwrap(),
            if index == 0 {
                hyperreal::RealSign::Negative
            } else {
                hyperreal::RealSign::Positive
            }
        );
        coordinate(&vector, Axis::X, &Real::one());
        coordinate(&vector, Axis::Z, &Real::zero());
        assert_eq!(
            vector.coordinate_sign(Axis::Y).unwrap(),
            if index == 0 {
                hyperreal::RealSign::Positive
            } else {
                hyperreal::RealSign::Negative
            }
        );
    }
}

pub fn cubic_and_projective_section_jets() {
    let equation = polynomial(vec![
        vec![0.into(), 0.into(), 0.into(), 1.into()],
        vec![(-1).into()],
    ]);
    for weighted in [false, true] {
        let weight = if weighted {
            polynomial(vec![vec![1.into()], vec![1.into()]])
        } else {
            one()
        };
        let surface = tensor([x(), y(), equation.clone()], weight);
        let sections = branches(plane().intersect_surface(&surface).unwrap());
        assert_eq!(sections.len(), 1);
        let section = &sections[0];
        let pcurve = section.first_pcurve().materialize().unwrap();
        let jets = pcurve.curve().derivatives_at(&q(1, 2).into(), 3).unwrap();
        let expected = if weighted {
            [0.into(), q(-2048, 19683), q(8388608, 531441)]
        } else {
            [q(4, 27), q(-256, 729), q(256000, 19683)]
        };
        for (jet, rhs) in jets.iter().zip(expected) {
            vector_power(jet, 3, &rhs);
        }
        for curve in [
            section.curve().clone(),
            archive_curve(
                section.curve(),
                if weighted {
                    "projective-cubic"
                } else {
                    "cubic"
                },
            ),
        ] {
            let xvalues = if weighted {
                [q(4, 9), q(-16, 27), q(32, 27)]
            } else {
                [1.into(), 0.into(), 0.into()]
            };
            for (index, value) in xvalues.into_iter().enumerate() {
                let v = curve.derivative_at(&q(1, 2).into(), index + 1).unwrap();
                coordinate(&v, Axis::X, &value);
                coordinate(&v, Axis::Z, &Real::zero());
                if weighted && index == 0 {
                    coordinate(&v, Axis::Y, &Real::zero());
                }
            }
        }
        let uv = section.second_pcurve().materialize().unwrap();
        vector_power(
            &uv.curve().derivative_at(&q(1, 2).into()).unwrap(),
            3,
            &q(4, 27),
        );
    }
}

pub fn smooth_vertical_events_and_singular_frontiers() {
    let sections = branches(
        plane()
            .intersect_surface(&tensor([x(), y(), circle()], one()))
            .unwrap(),
    );
    for (index, section) in sections.iter().enumerate() {
        for (end, parameter) in [
            section.curve().domain().start(),
            section.curve().domain().end(),
        ]
        .into_iter()
        .enumerate()
        {
            let result = section.curve().derivative_at(parameter, 1).unwrap_err();
            assert!(matches!(
                result,
                GeometryError::PlanarCurve(hypercurve::ExactCurveError::Blocked(_))
            ));
            let tangent = section.curve().tangent_direction_at(parameter).unwrap();
            coordinate(&tangent, Axis::X, &Real::zero());
            coordinate(&tangent, Axis::Z, &Real::zero());
            assert_eq!(
                tangent.coordinate_sign(Axis::Y).unwrap(),
                if index == end {
                    hyperreal::RealSign::Negative
                } else {
                    hyperreal::RealSign::Positive
                }
            );
            let reversed = section.reversed().unwrap();
            let reflected = reversed_parameter(section.curve(), reversed.curve(), parameter);
            assert_eq!(
                reversed
                    .curve()
                    .tangent_direction_at(&reflected)
                    .unwrap()
                    .coordinate_sign(Axis::Y)
                    .unwrap(),
                if index == end {
                    hyperreal::RealSign::Positive
                } else {
                    hyperreal::RealSign::Negative
                }
            );
            let restored = archive_curve(section.curve(), &format!("vertical-{index}-{end}"));
            assert_eq!(
                restored
                    .tangent_direction_at(parameter)
                    .unwrap()
                    .coordinate_sign(Axis::Y)
                    .unwrap(),
                tangent.coordinate_sign(Axis::Y).unwrap()
            );
        }
    }
    let node = polynomial(vec![
        vec![0.into(), (-1).into(), 1.into()],
        vec![1.into()],
        vec![(-1).into()],
    ]);
    let cusp = polynomial(vec![
        vec![q(3, 8), (-1).into(), 1.into()],
        vec![q(-3, 4)],
        vec![q(3, 2)],
        vec![(-1).into()],
    ]);
    for equation in [node, cusp] {
        let sections = branches(
            plane()
                .intersect_surface(&tensor([x(), y(), equation], one()))
                .unwrap(),
        );
        for section in sections {
            for result in [
                section.curve().derivative_at(&q(1, 2).into(), 1),
                section.curve().tangent_direction_at(&q(1, 2).into()),
            ] {
                assert!(matches!(
                    result,
                    Err(GeometryError::PlanarCurve(
                        hypercurve::ExactCurveError::Blocked(_)
                    ))
                ));
            }
        }
    }
    let source = sections[0].second_pcurve().materialize().unwrap();
    let collapsed = source
        .curve()
        .rational_image(&RationalMap2::new(
            [polynomial(vec![]), polynomial(vec![])],
            one(),
        ))
        .unwrap();
    let at: CurveParameter2 = q(3, 5).into();
    let zero = collapsed.derivative_at(&at).unwrap();
    assert_eq!(
        zero.coordinate_sign(hypercurve::Axis2::X).unwrap(),
        hyperreal::RealSign::Zero
    );
    assert_eq!(
        zero.coordinate_sign(hypercurve::Axis2::Y).unwrap(),
        hyperreal::RealSign::Zero
    );
    assert!(matches!(
        collapsed
            .polynomial_branch()
            .unwrap()
            .tangent_direction_at(&at),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
}

pub fn resource_domain_guards_and_policy_recovery() {
    let sections = branches(
        plane()
            .intersect_surface(&tensor([x(), y(), circle()], one()))
            .unwrap(),
    );
    let curve = sections[0].curve();
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| curve.derivative_at(&q(3, 5).into(), 2));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || check_circle(curve, 1));
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
    assert_eq!(
        curve.derivative_at(&q(3, 5).into(), 0).unwrap_err(),
        GeometryError::InvalidDerivativeOrder
    );
    assert_eq!(
        curve.derivative_at(&Real::zero().into(), 1).unwrap_err(),
        GeometryError::ParameterOutsideDomain
    );
    let pcurve = sections[0].second_pcurve().materialize().unwrap();
    let jets = pcurve.curve().derivatives_at(&q(3, 5).into(), 0).unwrap();
    assert!(jets.is_empty());
    assert!(matches!(
        pcurve.curve().derivatives_at(&q(3, 5).into(), usize::MAX),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let vector = pcurve.curve().derivative_at(&q(3, 5).into()).unwrap();
    let zero = polynomial(vec![vec![(-1).into()], vec![1.into()]]); // dx - 1 = 0
    assert!(matches!(
        vector.rational_form(zero.clone(), zero),
        Err(hypercurve::ExactCurveError::Invalid {
            cause: hypercurve::CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
    let stationary = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::origin(),
        ],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert!(
        stationary
            .derivative_at(&q(1, 2).into(), 1)
            .unwrap()
            .is_zero()
            .unwrap()
    );
    assert_eq!(
        stationary
            .tangent_direction_at(&q(1, 2).into())
            .unwrap_err(),
        GeometryError::UnsupportedSelectedEvaluation
    );
    check_circle(curve, 1);
}

pub fn selected_parallel_vector_signs() {
    let parameter = selected_parameter()
        .affine_image(&q(4, 3), &Real::zero())
        .unwrap();
    for (index, distance) in [q(1, 4), 4.into()].into_iter().enumerate() {
        // Source C(t)=(t,t^2), t^3=1/5. Its nonzero parallel speed ratio
        // is 1-2d/(1+4t^2)^(3/2), positive for d=1/4 and negative for d=4.
        // The d=4 cusp t=sqrt(3)/2 is outside [0,3/4].
        let parallel = hypercurve::QuadraticBezier2::new(
            hypercurve::Point2::from_values(0, 0),
            hypercurve::Point2::new(q(1, 2), 0.into()),
            hypercurve::Point2::from_values(1, 1),
        )
        .parallel_left(distance)
        .unwrap();
        let source = hypercurve::Curve2::try_analytic_parallel(
            parallel,
            hypercurve::BezierParameterRange2::try_new(
                hypercurve::BezierParameter2::Exact(Real::zero()),
                hypercurve::BezierParameter2::Exact(q(3, 4)),
            )
            .unwrap(),
        )
        .unwrap();
        let curve = Curve3::planar_image(
            source,
            Point3::origin(),
            Vector3::new([1.into(), 0.into(), 2.into()]),
            Vector3::new([0.into(), 1.into(), 3.into()]),
        )
        .unwrap();
        for curve in [
            curve.clone(),
            archive_curve(&curve, &format!("parallel-sign-{index}")),
        ] {
            let vector = curve.derivative_at(&parameter, 1).unwrap();
            assert!(vector.try_coordinates().is_none());
            for axis in [Axis::X, Axis::Y, Axis::Z] {
                assert_eq!(
                    vector.coordinate_sign(axis).unwrap(),
                    if index == 0 {
                        hyperreal::RealSign::Positive
                    } else {
                        hyperreal::RealSign::Negative
                    }
                );
            }
            assert_eq!(
                vector
                    .compare_linear_form(
                        &Vector3::new([(-2).into(), (-3).into(), 1.into()]),
                        &Real::zero()
                    )
                    .unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                vector
                    .compare_linear_form(
                        &Vector3::new([2.into(), (-1).into(), 0.into()]),
                        &Real::zero()
                    )
                    .unwrap(),
                if index == 0 {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            );
            assert!(matches!(
                vector.compare_linear_form(&Vector3::x(), &Real::one()),
                Err(GeometryError::PlanarCurve(
                    hypercurve::ExactCurveError::Blocked(_)
                ))
            ));
        }
    }
}
