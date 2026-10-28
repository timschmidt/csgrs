#![cfg(feature = "svg")]

use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CurvePoint2, CurvePointDefinition2, CurvePointLocations2,
    CurveRegion2, RationalMap2, Real, SvgGeometry2, export_svg_document, import_svg_document,
};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn center() -> CurvePoint2 {
    CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[-2, 0, 0, 0, 0, 1], &[1]]),
        parameter: BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![
                    (-1).into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    2.into(),
                ])
                .unwrap(),
                BezierParameterInterval::try_new(q(3, 4), 1.into()).unwrap(),
            )
            .unwrap(),
        ),
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
fn translate(point: &CurvePoint2, x: Real, y: Real) -> CurvePoint2 {
    point
        .rational_image(&RationalMap2::new(
            [
                BivariatePolynomial::new(vec![vec![x], vec![1.into()]]),
                BivariatePolynomial::new(vec![vec![y, 1.into()]]),
            ],
            polynomial(&[&[1]]),
        ))
        .unwrap()
}
fn real(bytes: &mut Vec<u8>, value: Real) {
    let encoded = value.to_bytes();
    bytes.extend_from_slice(&u32::try_from(encoded.len()).unwrap().to_be_bytes());
    bytes.extend(encoded);
}
fn record(tag: u8, references: &[u32]) -> Vec<u8> {
    let mut bytes = vec![tag];
    for reference in references {
        bytes.extend_from_slice(&reference.to_be_bytes());
    }
    bytes
}
fn point(x: i32, y: i32) -> Vec<u8> {
    let mut bytes = vec![7];
    real(&mut bytes, x.into());
    real(&mut bytes, y.into());
    bytes
}
fn chord(tag: u8, references: &[u32]) -> Vec<u8> {
    let mut bytes = record(tag, references);
    bytes.extend([0, 1]);
    bytes
}
fn circle_path() -> hypercurve::CurvePath2 {
    circle_path_in_frame(1, true)
}
fn circle_path_in_frame(radius: i32, clockwise: bool) -> hypercurve::CurvePath2 {
    use std::fmt::Write;
    let equation = polynomial(&[&[-2, 0, 0, 0, 0, 1], &[1]]);
    let mut root = vec![2];
    root.extend_from_slice(&8_u32.to_be_bytes());
    for i in 0..8 {
        real(
            &mut root,
            if i == 0 {
                (-1).into()
            } else if i == 7 {
                2.into()
            } else {
                0.into()
            },
        );
    }
    real(&mut root, q(3, 4));
    real(&mut root, 1.into());
    let mut fiber = record(14, &[0]);
    fiber.extend_from_slice(
        &u32::try_from(equation.coefficients.len())
            .unwrap()
            .to_be_bytes(),
    );
    for row in &equation.coefficients {
        fiber.extend_from_slice(&u32::try_from(row.len()).unwrap().to_be_bytes());
        for value in row {
            real(&mut fiber, value.clone());
        }
    }
    real(&mut fiber, 1.into());
    real(&mut fiber, 2.into());
    fiber.push(0);
    let mut circle = record(20, &[1, 5]);
    real(&mut circle, radius.into());
    circle.push(u8::from(clockwise));
    let mut start = record(21, &[6]);
    real(&mut start, 0.into());
    let mut end = record(21, &[6]);
    real(&mut end, 1.into());
    let mut range = record(23, &[6, 7, 8]);
    range.push(0);
    let nodes = vec![
        root,
        fiber,
        point(0, 0),
        point(3, 4),
        chord(11, &[2, 3]),
        chord(12, &[4, 2, 3]),
        circle,
        start,
        end,
        range,
    ];
    let mut bytes = b"HCP4".to_vec();
    bytes.extend_from_slice(&u32::try_from(nodes.len()).unwrap().to_be_bytes());
    bytes.extend(nodes.into_iter().flatten());
    bytes.extend_from_slice(&1_u32.to_be_bytes());
    bytes.extend_from_slice(&9_u32.to_be_bytes());
    let mut metadata = String::from("4:");
    for byte in bytes {
        write!(&mut metadata, "{byte:02x}").unwrap();
    }
    let svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="{metadata}" d="M0 0 L1 1"/></svg>"#
    );
    import_svg_document(&svg).unwrap().paths()[0].clone()
}

#[test]
fn common_field_rational_queries_certify_the_independent_radius() {
    let center = center();
    assert!(center.coordinates().is_none());
    let p = translate(&center, q(3, 5), q(4, 5));
    let coordinate = |p: &CurvePoint2, axis| {
        p.rational_form(
            if axis == 0 {
                polynomial(&[&[0], &[1]])
            } else {
                polynomial(&[&[0, 1]])
            },
            polynomial(&[&[1]]),
        )
        .unwrap()
    };
    let dx = coordinate(&p, 0).subtract(&coordinate(&center, 0)).unwrap();
    let dy = coordinate(&p, 1).subtract(&coordinate(&center, 1)).unwrap();
    let radius = dx
        .multiply(&dx)
        .unwrap()
        .add(&dy.multiply(&dy).unwrap())
        .unwrap();
    assert_eq!(
        radius.compare_to_real(&Real::one()).unwrap(),
        Ordering::Equal
    );
}

#[test]
fn oblique_linear_queries_retain_correlated_fiber_cancellation() {
    let center = center();
    let p = translate(&center, q(3, 5), q(4, 5));
    for (a, b, expected) in [
        (-4, 3, Ordering::Equal),
        (3, 4, Ordering::Greater),
        (-3, -4, Ordering::Less),
    ] {
        let first = p.affine_form(a.into(), b.into(), 0.into());
        let second = center.affine_form(a.into(), b.into(), 0.into());
        assert_eq!(first.compare(&second).unwrap(), expected);
        assert_eq!(second.compare(&first).unwrap(), expected.reverse());
    }
}

#[test]
fn correlated_fiber_circle_midpoint_reenters_native_replay() {
    let center = center();
    let path = circle_path();
    let curve = &path.curves()[0];
    assert!(
        curve
            .start()
            .coincides_with(&translate(&center, q(-4, 5), q(3, 5)))
            .unwrap()
    );
    assert!(
        curve
            .end()
            .coincides_with(&translate(&center, q(4, 5), q(-3, 5)))
            .unwrap()
    );
    let midpoint = translate(&center, q(3, 5), q(4, 5));
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&midpoint).unwrap()
    else {
        panic!("one midpoint")
    };
    assert_eq!(locations.len(), 1);
    assert!(
        curve
            .point_at(&locations[0].parameter().unwrap())
            .unwrap()
            .coincides_with(&midpoint)
            .unwrap()
    );
    let encode = |path| {
        export_svg_document(&SvgGeometry2::new(
            CurveRegion2::empty(),
            vec![],
            vec![path],
        ))
        .unwrap()
    };
    let (first, second) = curve.split_at(locations[0].parameter().unwrap()).unwrap();
    assert!(first.end().coincides_with(&midpoint).unwrap());
    assert!(second.start().coincides_with(&midpoint).unwrap());
    let reversed = hypercurve::CurvePath2::try_new(vec![
        second.reversed().unwrap(),
        first.reversed().unwrap(),
    ])
    .unwrap();
    let reversed_text = encode(reversed);
    let twice = import_svg_document(&reversed_text).unwrap();
    assert!(
        twice.paths()[0]
            .start()
            .coincides_with(&curve.end())
            .unwrap()
    );
    assert!(
        twice.paths()[0]
            .end()
            .coincides_with(&curve.start())
            .unwrap()
    );
    assert_eq!(encode(twice.paths()[0].clone()), reversed_text);
    let text = encode(path);
    assert_eq!(
        encode(import_svg_document(&text).unwrap().paths()[0].clone()),
        text
    );
}

#[test]
fn correlated_circle_queries_preserve_radius_sign_half_and_exact_separation() {
    let center = center();
    for radius in [-1, 1] {
        for clockwise in [false, true] {
            let path = circle_path_in_frame(radius, clockwise);
            let curve = &path.curves()[0];
            let direction = if clockwise { radius } else { -radius };
            let midpoint = translate(&center, q(3 * direction, 5), q(4 * direction, 5));
            let opposite = translate(&center, q(-3 * direction, 5), q(-4 * direction, 5));
            let quarter = translate(
                &center,
                q(-16 * radius + 9 * direction, 25),
                q(12 * radius + 12 * direction, 25),
            );
            for point in [curve.start(), quarter, midpoint.clone(), curve.end()] {
                let CurvePointLocations2::Locations(locations) =
                    curve.point_locations(&point).unwrap()
                else {
                    panic!("one finite arc point")
                };
                assert_eq!(locations.len(), 1);
                assert!(
                    curve
                        .point_at(&locations[0].parameter().unwrap())
                        .unwrap()
                        .coincides_with(&point)
                        .unwrap()
                );
            }
            let delta = (Real::one() / Real::from(1_u128 << 80)).unwrap();
            let near = translate(&midpoint, delta, Real::zero());
            for point in [center.clone(), opposite, near] {
                let CurvePointLocations2::Locations(locations) =
                    curve.point_locations(&point).unwrap()
                else {
                    panic!("finite nonincident point")
                };
                assert!(locations.is_empty());
            }
        }
    }
}

#[test]
fn correlated_circle_queries_survive_algebraic_similarity_and_replay() {
    let path = circle_path();
    let independent = translate(&center(), q(-7, 25), q(24, 25));
    let root = Real::from(2).sqrt().unwrap();
    for transform in [
        hypercurve::Similarity2::try_from_real_affine(
            root.clone(),
            -root.clone(),
            root.clone(),
            root.clone(),
            3.into(),
            (-2).into(),
        )
        .unwrap(),
        hypercurve::Similarity2::try_from_real_affine(
            -root.clone(),
            0.into(),
            0.into(),
            root,
            1.into(),
            4.into(),
        )
        .unwrap(),
    ] {
        let transformed = path.transform_similarity(&transform).unwrap();
        let point = CurvePointDefinition2::Similarity {
            source: independent.clone(),
            transform,
        }
        .into_point()
        .unwrap();
        let text = export_svg_document(&SvgGeometry2::new(
            CurveRegion2::empty(),
            vec![],
            vec![transformed.clone()],
        ))
        .unwrap();
        let replay = import_svg_document(&text).unwrap();
        let curve = &replay.paths()[0].curves()[0];
        assert!(curve.start().coincides_with(&transformed.start()).unwrap());
        assert!(curve.end().coincides_with(&transformed.end()).unwrap());
        let CurvePointLocations2::Locations(locations) = curve.point_locations(&point).unwrap()
        else {
            panic!("one transformed circle point")
        };
        assert_eq!(locations.len(), 1);
        assert!(
            curve
                .point_at(&locations[0].parameter().unwrap())
                .unwrap()
                .coincides_with(&point)
                .unwrap()
        );
        assert_eq!(export_svg_document(&replay).unwrap(), text);
    }
}

#[test]
fn common_field_circle_queries_keep_negative_denominators_and_original_poles() {
    let center = center();
    let midpoint = center
        .rational_image(&RationalMap2::new(
            [
                BivariatePolynomial::new(vec![vec![q(-6, 5)], vec![(-2).into()]]),
                BivariatePolynomial::new(vec![vec![q(-8, 5), (-2).into()]]),
            ],
            polynomial(&[&[-2]]),
        ))
        .unwrap();
    assert!(
        midpoint
            .coincides_with(&translate(&center, q(3, 5), q(4, 5)))
            .unwrap()
    );
    assert_eq!(
        midpoint
            .affine_form((-4).into(), 3.into(), 0.into())
            .compare(&center.affine_form((-4).into(), 3.into(), 0.into()))
            .unwrap(),
        Ordering::Equal
    );
    let path = circle_path();
    let CurvePointLocations2::Locations(locations) =
        path.curves()[0].point_locations(&midpoint).unwrap()
    else {
        panic!("one exact midpoint")
    };
    assert_eq!(locations.len(), 1);
    let guard = polynomial(&[&[-2, 0, 0, 0, 0, 1], &[1]]);
    assert!(matches!(
        center.rational_image(&RationalMap2::new([guard.clone(), guard.clone()], guard)),
        Err(hypercurve::ExactCurveError::Invalid {
            cause: hypercurve::CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
}

#[test]
fn correlated_circle_queries_retry_after_counted_exhaustion_without_policy_upgrade() {
    let path = circle_path();
    let midpoint = translate(&center(), q(3, 5), q(4, 5));
    let query = || path.curves()[0].point_locations(&midpoint);
    for _ in 0..2 {
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, query);
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 512,
                promotions: 0,
            },
            query,
        );
        let CurvePointLocations2::Locations(locations) = retry.result.unwrap().unwrap() else {
            panic!("one midpoint after retry")
        };
        assert_eq!(locations.len(), 1);
        assert!(retry.usage.refinements <= 512);
        assert_eq!(retry.usage.promotions, 0);
    }
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let CurvePointLocations2::Locations(locations) = query().unwrap() else {
                panic!("one exact midpoint")
            };
            assert_eq!(locations.len(), 1);
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
}

#[test]
fn nonconstant_angular_point_reenters_selected_circle() {
    let center = center();
    // u=beta in the rational half-circle chart, D=1-2u+2u^2.
    // N=(-4/5,3/5), T=(3/5,4/5), n=(1-2u)/D,
    // b=2u(1-u)/D, so P=C+nN+bT is independently on the unit circle.
    let point = center
        .rational_image(&RationalMap2::new(
            [
                BivariatePolynomial::new(vec![
                    vec![q(-4, 5)],
                    vec![q(19, 5)],
                    vec![q(-16, 5)],
                    vec![2.into()],
                ]),
                BivariatePolynomial::new(vec![
                    vec![q(3, 5), 1.into()],
                    vec![q(2, 5), (-2).into()],
                    vec![q(-8, 5), 2.into()],
                ]),
            ],
            polynomial(&[&[1], &[-2], &[2]]),
        ))
        .unwrap();
    assert!(point.coordinates().is_none());
    let coordinate = |p: &CurvePoint2, axis| {
        p.rational_form(
            if axis == 0 {
                polynomial(&[&[0], &[1]])
            } else {
                polynomial(&[&[0, 1]])
            },
            polynomial(&[&[1]]),
        )
        .unwrap()
    };
    let dx = coordinate(&point, 0)
        .subtract(&coordinate(&center, 0))
        .unwrap();
    let dy = coordinate(&point, 1)
        .subtract(&coordinate(&center, 1))
        .unwrap();
    assert_eq!(
        dx.multiply(&dx)
            .unwrap()
            .add(&dy.multiply(&dy).unwrap())
            .unwrap()
            .compare_to_real(&Real::one())
            .unwrap(),
        Ordering::Equal
    );
    let path = circle_path();
    let curve = &path.curves()[0];
    let CurvePointLocations2::Locations(locations) = curve.point_locations(&point).unwrap() else {
        panic!("one selected algebraic angular point")
    };
    assert_eq!(locations.len(), 1);
    let parameter = locations[0].parameter().unwrap();
    assert!(
        curve
            .point_at(&parameter)
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    let (first, second) = curve.split_at(parameter).unwrap();
    let reversed = hypercurve::CurvePath2::try_new(vec![
        second.reversed().unwrap(),
        first.reversed().unwrap(),
    ])
    .unwrap();
    let svg = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![reversed],
    ))
    .unwrap();
    assert_eq!(
        export_svg_document(&import_svg_document(&svg).unwrap()).unwrap(),
        svg
    );
}

fn algebraic_angular_point(radius: i32, clockwise: bool, scale: Real, shift: Real) -> CurvePoint2 {
    use hypersolve::bivariate_arithmetic::{
        bivariate_add, bivariate_multiply, bivariate_scale, bivariate_subtract,
    };
    let u = BivariatePolynomial::new(vec![vec![shift], vec![scale]]);
    let twice = bivariate_scale(u.clone(), &Real::from(2));
    let square = bivariate_scale(bivariate_multiply(&u, &u), &Real::from(2));
    let normal = bivariate_subtract(&polynomial(&[&[1]]), &twice);
    let tangent = bivariate_subtract(&twice, &square);
    let denominator = bivariate_add(&normal, &square);
    let direction = if clockwise { radius } else { -radius };
    let x = bivariate_add(
        &bivariate_multiply(&polynomial(&[&[0], &[1]]), &denominator),
        &bivariate_add(
            &bivariate_scale(normal.clone(), &q(-4 * radius, 5)),
            &bivariate_scale(tangent.clone(), &q(3 * direction, 5)),
        ),
    );
    let y = bivariate_add(
        &bivariate_multiply(&polynomial(&[&[0, 1]]), &denominator),
        &bivariate_add(
            &bivariate_scale(normal, &q(3 * radius, 5)),
            &bivariate_scale(tangent, &q(4 * direction, 5)),
        ),
    );
    center()
        .rational_image(&RationalMap2::new([x, y], denominator))
        .unwrap()
}

fn angular_location(
    curve: &hypercurve::Curve2,
    point: &CurvePoint2,
) -> hypercurve::CurveParameter2 {
    let CurvePointLocations2::Locations(locations) = curve.point_locations(point).unwrap() else {
        panic!("one selected angular location")
    };
    assert_eq!(locations.len(), 1);
    locations[0].parameter().unwrap()
}

fn angular_svg(path: hypercurve::CurvePath2) -> String {
    export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![path],
    ))
    .unwrap()
}

#[test]
fn selected_circle_points_order_independent_definitions_and_nearby_cuts() {
    let path = circle_path();
    let curve = &path.curves()[0];
    let delta = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let points = [-delta.clone(), Real::zero(), delta]
        .map(|shift| algebraic_angular_point(1, true, Real::one(), shift));
    let parameters = points
        .each_ref()
        .map(|point| angular_location(curve, point));
    for index in 0..2 {
        assert_eq!(
            parameters[index].compare(&parameters[index + 1]).unwrap(),
            Ordering::Less
        );
        assert_eq!(
            parameters[index + 1].compare(&parameters[index]).unwrap(),
            Ordering::Greater
        );
        assert!(!points[index].coincides_with(&points[index + 1]).unwrap());
    }
    let independent = algebraic_angular_point(1, true, Real::one(), Real::zero());
    assert!(independent.coordinates().is_none());
    assert_eq!(
        parameters[1]
            .compare(&angular_location(curve, &independent))
            .unwrap(),
        Ordering::Equal
    );
    let range =
        hypercurve::CurveParameterRange2::try_new(parameters[0].clone(), parameters[2].clone())
            .unwrap();
    let restricted = curve.restrict_source(range).unwrap();
    assert!(restricted.start().coincides_with(&points[0]).unwrap());
    assert!(restricted.end().coincides_with(&points[2]).unwrap());
    let middle = angular_location(&restricted, &points[1]);
    let (left, right) = restricted.split_at(middle).unwrap();
    let reversed =
        hypercurve::CurvePath2::try_new(vec![right.reversed().unwrap(), left.reversed().unwrap()])
            .unwrap();
    let svg = angular_svg(reversed);
    let replay = import_svg_document(&svg).unwrap();
    assert!(
        replay.paths()[0]
            .start()
            .coincides_with(&points[2])
            .unwrap()
    );
    assert!(replay.paths()[0].end().coincides_with(&points[0]).unwrap());
    assert_eq!(angular_svg(replay.paths()[0].clone()), svg);
}

#[test]
fn selected_circle_points_keep_signed_halves_and_similarity_transport() {
    for radius in [-1, 1] {
        for clockwise in [false, true] {
            let path = circle_path_in_frame(radius, clockwise);
            let point = algebraic_angular_point(radius, clockwise, q(1, 2), q(1, 4));
            let opposite = algebraic_angular_point(-radius, clockwise, q(1, 2), q(1, 4));
            let curve = &path.curves()[0];
            let parameter = angular_location(curve, &point);
            assert!(
                curve
                    .point_at(&parameter)
                    .unwrap()
                    .coincides_with(&point)
                    .unwrap()
            );
            let near = translate(
                &point,
                (Real::one() / Real::from(1_u128 << 80)).unwrap(),
                Real::zero(),
            );
            for excluded in [opposite, near] {
                let CurvePointLocations2::Locations(locations) =
                    curve.point_locations(&excluded).unwrap()
                else {
                    panic!("finite excluded point")
                };
                assert!(locations.is_empty());
            }
            let (first, second) = curve.split_at(parameter).unwrap();
            let split = hypercurve::CurvePath2::try_new(vec![first, second]).unwrap();
            let root = Real::from(2).sqrt().unwrap();
            for transform in [
                hypercurve::Similarity2::try_from_real_affine(
                    root.clone(),
                    -root.clone(),
                    root.clone(),
                    root.clone(),
                    3.into(),
                    (-2).into(),
                )
                .unwrap(),
                hypercurve::Similarity2::try_from_real_affine(
                    -root.clone(),
                    0.into(),
                    0.into(),
                    root,
                    1.into(),
                    4.into(),
                )
                .unwrap(),
            ] {
                let transformed = split.transform_similarity(&transform).unwrap();
                let expected = CurvePointDefinition2::Similarity {
                    source: point.clone(),
                    transform,
                }
                .into_point()
                .unwrap();
                let svg = angular_svg(transformed);
                let replay = import_svg_document(&svg).unwrap();
                for piece in replay.paths()[0].curves() {
                    let parameter = angular_location(piece, &expected);
                    assert!(
                        piece
                            .point_at(&parameter)
                            .unwrap()
                            .coincides_with(&expected)
                            .unwrap()
                    );
                }
                assert_eq!(angular_svg(replay.paths()[0].clone()), svg);
            }
        }
    }
}

#[test]
fn selected_circle_points_replay_in_a_fresh_process_before_another_split() {
    const INPUT: &str = "HYPERCURVE_SELECTED_CIRCLE_POINTS_REPLAY_INPUT";
    let verify = |text: &str| {
        let replay = import_svg_document(text).unwrap();
        let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
        assert!(
            replay.paths()[0].curves()[0]
                .end()
                .coincides_with(&point)
                .unwrap()
        );
        assert!(
            replay.paths()[0].curves()[1]
                .start()
                .coincides_with(&point)
                .unwrap()
        );
        let next = algebraic_angular_point(1, true, q(1, 2), Real::zero());
        let curve = &replay.paths()[0].curves()[1];
        let parameter = angular_location(curve, &next);
        let (first, second) = curve.split_at(parameter).unwrap();
        assert!(first.end().coincides_with(&next).unwrap());
        assert!(second.start().coincides_with(&next).unwrap());
        let next_svg = angular_svg(hypercurve::CurvePath2::try_new(vec![first, second]).unwrap());
        assert_eq!(
            export_svg_document(&import_svg_document(&next_svg).unwrap()).unwrap(),
            next_svg
        );
        assert_eq!(export_svg_document(&replay).unwrap(), text);
    };
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let path = circle_path();
    let curve = &path.curves()[0];
    let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let (first, second) = curve.split_at(angular_location(curve, &point)).unwrap();
    let text = angular_svg(
        hypercurve::CurvePath2::try_new(vec![
            second.reversed().unwrap(),
            first.reversed().unwrap(),
        ])
        .unwrap(),
    );
    verify(&text);
    let file = std::env::temp_dir().join(format!(
        "hypercurve-selected-circle-points-{}.svg",
        std::process::id()
    ));
    std::fs::write(&file, &text).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_circle_points_replay_in_a_fresh_process_before_another_split",
            "--nocapture",
        ])
        .env(INPUT, &file)
        .output()
        .unwrap();
    std::fs::remove_file(file).unwrap();
    assert!(
        output.status.success(),
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn selected_circle_point_queries_retry_without_promoting_requested_policy() {
    let path = circle_path();
    let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let query = || path.curves()[0].point_locations(&point);
    for _ in 0..2 {
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, query);
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            query,
        );
        let CurvePointLocations2::Locations(locations) = retry.result.unwrap().unwrap() else {
            panic!("one point after retry")
        };
        assert_eq!(locations.len(), 1);
        assert_eq!(retry.usage.promotions, 0);
    }
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let parameter = angular_location(&path.curves()[0], &point);
            let (first, second) = path.curves()[0].split_at(parameter).unwrap();
            let text = angular_svg(hypercurve::CurvePath2::try_new(vec![first, second]).unwrap());
            assert_eq!(
                export_svg_document(&import_svg_document(&text).unwrap()).unwrap(),
                text
            );
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
}

#[test]
fn selected_circle_point_derivative_keeps_joint_field_and_exact_magnitude() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    let path = circle_path();
    let curve = &path.curves()[0];
    let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let parameter = angular_location(curve, &point);
    let denominator = polynomial(&[&[1], &[-2], &[2]]);
    // Differentiate the rational circle chart while holding its center fixed.
    // n'=-4u(1-u)/D^2, b'=2(1-2u)/D^2, N=(-4/5,3/5), T=(3/5,4/5).
    let expected = center()
        .rational_image(&RationalMap2::new(
            [
                BivariatePolynomial::new(vec![vec![q(6, 5)], vec![q(4, 5)], vec![q(-16, 5)]]),
                BivariatePolynomial::new(vec![vec![q(8, 5)], vec![q(-28, 5)], vec![q(12, 5)]]),
            ],
            bivariate_multiply(&denominator, &denominator),
        ))
        .unwrap();
    let derivative = curve.derivative_at(&parameter).unwrap();
    assert!(
        derivative
            .coordinate_image()
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
}

// Independent closed form: z=(1-u+iu)/(1-u-iu), so for k>=1,
// z^(k)=2i*k!*(1+i)^(k-1)/(1-(1+i)u)^(k+1).
// Rationalizing this expression supplies all orders without the evaluator's
// Leibniz recurrence or differentiation of a reduced selected-field value.
fn circle_derivative_expectation(
    order: usize,
    radius: i32,
    clockwise: bool,
    scale: Real,
    shift: Real,
) -> RationalMap2 {
    use hypersolve::bivariate_arithmetic::{
        bivariate_add as add, bivariate_multiply as mul, bivariate_scale as times,
        bivariate_subtract as sub,
    };
    let u = BivariatePolynomial::new(vec![vec![shift], vec![scale]]);
    let a = sub(&polynomial(&[&[1]]), &u);
    let d = add(&mul(&a, &a), &mul(&u, &u));
    let complex_product = |left: &[BivariatePolynomial; 2], right: &[BivariatePolynomial; 2]| {
        [
            sub(&mul(&left[0], &right[0]), &mul(&left[1], &right[1])),
            add(&mul(&left[0], &right[1]), &mul(&left[1], &right[0])),
        ]
    };
    let mut constant = [polynomial(&[&[0]]), polynomial(&[&[2]])];
    for k in 1..=order {
        if k > 1 {
            constant = complex_product(&constant, &[polynomial(&[&[1]]), polynomial(&[&[1]])]);
        }
        constant = constant.map(|value| times(value, &Real::from(u128::try_from(k).unwrap())));
    }
    let mut numerator = constant;
    let mut denominator = polynomial(&[&[1]]);
    for _ in 0..=order {
        numerator = complex_product(&numerator, &[a.clone(), u.clone()]);
        denominator = mul(&denominator, &d);
    }
    let direction = if clockwise { radius } else { -radius };
    RationalMap2::new(
        [
            add(
                &times(numerator[0].clone(), &q(-4 * radius, 5)),
                &times(numerator[1].clone(), &q(3 * direction, 5)),
            ),
            add(
                &times(numerator[0].clone(), &q(3 * radius, 5)),
                &times(numerator[1].clone(), &q(4 * direction, 5)),
            ),
        ],
        denominator,
    )
}

#[test]
fn selected_circle_derivatives_match_independent_complex_formula() {
    for (radius, clockwise) in [(1, true), (-2, false)] {
        let path = circle_path_in_frame(radius, clockwise);
        let point = algebraic_angular_point(radius, clockwise, q(1, 2), q(1, 4));
        let parameter = angular_location(&path.curves()[0], &point);
        let derivatives = path.curves()[0].derivatives_at(&parameter, 3).unwrap();
        assert_eq!(derivatives.len(), 3);
        for (index, derivative) in derivatives.iter().enumerate() {
            let expected = center()
                .rational_image(&circle_derivative_expectation(
                    index + 1,
                    radius,
                    clockwise,
                    q(1, 2),
                    q(1, 4),
                ))
                .unwrap();
            let image = derivative.coordinate_image().unwrap();
            assert!(image.coordinates().is_none());
            assert!(
                image.coincides_with(&expected).unwrap(),
                "order {}",
                index + 1
            );
            let replay = image.definition().unwrap().into_point().unwrap();
            assert!(replay.coincides_with(&expected).unwrap());
        }
    }
}

#[test]
fn selected_circle_scalar_derivatives_keep_chart_endpoints_and_orders() {
    for radius in [-2, 1] {
        for clockwise in [false, true] {
            let path = circle_path_in_frame(radius, clockwise);
            let curve = &path.curves()[0];
            for value in [Real::zero(), q(1, 2), Real::one()] {
                let parameter = hypercurve::CurveParameter2::from(value.clone());
                let derivatives = curve.derivatives_at(&parameter, 8).unwrap();
                assert_eq!(derivatives.len(), 8);
                for (index, derivative) in derivatives.iter().enumerate() {
                    let source: CurvePoint2 =
                        hypercurve::Point2::new(value.clone(), Real::zero()).into();
                    let expected = source
                        .rational_image(&circle_derivative_expectation(
                            index + 1,
                            radius,
                            clockwise,
                            Real::one(),
                            Real::zero(),
                        ))
                        .unwrap();
                    assert!(
                        derivative
                            .coordinate_image()
                            .unwrap()
                            .coincides_with(&expected)
                            .unwrap()
                    );
                }
                assert!(curve.derivatives_at(&parameter, 0).unwrap().is_empty());
            }
            for outside in [-1, 2] {
                assert!(
                    curve
                        .derivatives_at(&Real::from(outside).into(), 0)
                        .is_err()
                );
                assert!(
                    curve
                        .derivatives_at(&Real::from(outside).into(), 3)
                        .is_err()
                );
            }
        }
    }
}

#[test]
fn selected_circle_derivatives_survive_restriction_reversal_and_similarity() {
    let path = circle_path();
    let point = algebraic_angular_point(1, true, q(1, 2), q(1, 4));
    let parameter = angular_location(&path.curves()[0], &point);
    let expected = center()
        .rational_image(&circle_derivative_expectation(1, 1, true, q(1, 2), q(1, 4)))
        .unwrap();
    let (left, right) = path.curves()[0].split_at(parameter).unwrap();
    let split =
        hypercurve::CurvePath2::try_new(vec![right.reversed().unwrap(), left.reversed().unwrap()])
            .unwrap();
    // Retained ranges use the original source chart, including reversed traversal.
    for piece in split.curves() {
        let at = angular_location(piece, &point);
        assert!(
            piece
                .derivative_at(&at)
                .unwrap()
                .coordinate_image()
                .unwrap()
                .coincides_with(&expected)
                .unwrap()
        );
    }
    let root = Real::from(2).sqrt().unwrap();
    for (a, b, c, d) in [
        (root.clone(), -root.clone(), root.clone(), root.clone()),
        (-root.clone(), Real::zero(), Real::zero(), root),
    ] {
        let transform = hypercurve::Similarity2::try_from_real_affine(
            a.clone(),
            b.clone(),
            c.clone(),
            d.clone(),
            3.into(),
            (-2).into(),
        )
        .unwrap();
        let linear =
            hypercurve::Similarity2::try_from_real_affine(a, b, c, d, 0.into(), 0.into()).unwrap();
        let target = CurvePointDefinition2::Similarity {
            source: point.clone(),
            transform: transform.clone(),
        }
        .into_point()
        .unwrap();
        let expected = CurvePointDefinition2::Similarity {
            source: expected.clone(),
            transform: linear,
        }
        .into_point()
        .unwrap();
        let text = angular_svg(split.transform_similarity(&transform).unwrap());
        let replay = import_svg_document(&text).unwrap();
        for piece in replay.paths()[0].curves() {
            let at = angular_location(piece, &target);
            assert!(
                piece
                    .derivative_at(&at)
                    .unwrap()
                    .coordinate_image()
                    .unwrap()
                    .coincides_with(&expected)
                    .unwrap()
            );
        }
        assert_eq!(angular_svg(replay.paths()[0].clone()), text);
    }
}

#[test]
fn selected_circle_derivatives_retry_and_keep_requested_policy_certainty() {
    let path = circle_path();
    let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let parameter = angular_location(&path.curves()[0], &point);
    let query = || path.curves()[0].derivative_at(&parameter);
    // This retained exact derivative needs no charged refinement or promotion.
    // Zero budget must permit it; exhaustion may only report work actually denied.
    let exact =
        hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, query);
    assert!(exact.result.unwrap().unwrap().coordinate_image().is_ok());
    assert_eq!((exact.usage.refinements, exact.usage.promotions), (0, 0));
    let compare = || query()?.compare_linear_form(&Real::one(), &Real::zero(), &Real::from(-1));
    let exact_compare =
        hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, compare);
    assert_eq!(exact_compare.result.unwrap().unwrap(), Ordering::Less);
    assert_eq!(
        (
            exact_compare.usage.refinements,
            exact_compare.usage.promotions
        ),
        (0, 0)
    );
    // The whole incidence -> derivative -> vector-query chain requires charged
    // work in incidence. Retained rational arithmetic itself has no total-work
    // bound from these counters; its successful ZERO queries above remain valid.
    let chain = || {
        let CurvePointLocations2::Locations(locations) =
            path.curves()[0].point_locations(&point)?
        else {
            panic!("one exact angular location");
        };
        assert_eq!(locations.len(), 1);
        path.curves()[0]
            .derivative_at(&locations[0].parameter().unwrap())?
            .compare_linear_form(&Real::one(), &Real::zero(), &Real::from(-1))
    };
    for _ in 0..2 {
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, chain);
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            chain,
        );
        assert_eq!(retry.result.unwrap().unwrap(), Ordering::Less);
        assert_eq!(retry.usage.promotions, 0);
    }
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let vector = query().unwrap();
            assert!(
                vector
                    .coordinate_image()
                    .unwrap()
                    .definition()
                    .unwrap()
                    .into_point()
                    .is_ok()
            );
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
}

#[test]
fn selected_circle_derivative_size_guard() {
    let path = circle_path();
    assert!(matches!(
        path.curves()[0].derivatives_at(&Real::zero().into(), usize::MAX),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Unsupported
    ));
}

#[test]
fn selected_circle_derivative_points_replay_before_another_exact_split() {
    const INPUT: &str = "HYPERCURVE_CIRCLE_DIFFERENTIAL_REPLAY_INPUT";
    let verify = |text: &str| {
        let replay = import_svg_document(text).unwrap();
        let expected = center()
            .rational_image(&circle_derivative_expectation(
                1,
                1,
                true,
                Real::one(),
                Real::zero(),
            ))
            .unwrap();
        let line = &replay.paths()[0].curves()[0];
        assert!(line.start().coincides_with(&expected).unwrap());
        assert!(
            line.end()
                .coincides_with(&translate(&expected, Real::one(), Real::zero()))
                .unwrap()
        );
        let midpoint = translate(&expected, q(1, 2), Real::zero());
        let at = angular_location(line, &midpoint);
        let (first, second) = line.split_at(at).unwrap();
        assert!(first.end().coincides_with(&midpoint).unwrap());
        assert!(second.start().coincides_with(&midpoint).unwrap());
        let next = angular_svg(hypercurve::CurvePath2::try_new(vec![first, second]).unwrap());
        assert_eq!(
            export_svg_document(&import_svg_document(&next).unwrap()).unwrap(),
            next
        );
        assert_eq!(export_svg_document(&replay).unwrap(), text);
    };
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let path = circle_path();
    let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let parameter = angular_location(&path.curves()[0], &point);
    let vector = path.curves()[0]
        .derivative_at(&parameter)
        .unwrap()
        .coordinate_image()
        .unwrap();
    let line = hypercurve::Curve2::try_line(
        vector.clone(),
        translate(&vector, Real::one(), Real::zero()),
    )
    .unwrap();
    let text = angular_svg(hypercurve::CurvePath2::try_new(vec![line]).unwrap());
    verify(&text);
    let file = std::env::temp_dir().join(format!(
        "hypercurve-circle-differential-{}.svg",
        std::process::id()
    ));
    std::fs::write(&file, &text).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_circle_derivative_points_replay_before_another_exact_split",
            "--nocapture",
        ])
        .env(INPUT, &file)
        .output()
        .unwrap();
    std::fs::remove_file(file).unwrap();
    assert!(
        output.status.success(),
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

fn rational_point_line_across_coarse_pole() -> hypercurve::CurvePath2 {
    // The selected beta lies above 4/5, but its authored isolator [3/4,1]
    // contains that pole. Refining the retained selection is sufficient;
    // the exact point itself has a nonzero original denominator.
    let start = center()
        .rational_image(&RationalMap2::new(
            [polynomial(&[&[1]]), polynomial(&[&[0, 1]])],
            BivariatePolynomial::new(vec![vec![q(-4, 5)], vec![1.into()]]),
        ))
        .unwrap();
    let end = translate(&start, Real::one(), Real::zero());
    hypercurve::CurvePath2::try_new(vec![hypercurve::Curve2::try_line(start, end).unwrap()])
        .unwrap()
}

#[test]
fn retained_rational_point_bounds_refine_original_pole_guards() {
    let path = rational_point_line_across_coarse_pole();
    let bounds = path.bounds().unwrap();
    let line = &path.curves()[0];
    for point in [line.start(), line.end()] {
        for (a, b, lo, hi) in [
            (Real::one(), Real::zero(), bounds.min_x(), bounds.max_x()),
            (Real::zero(), Real::one(), bounds.min_y(), bounds.max_y()),
        ] {
            let value = point.affine_form(a, b, Real::zero());
            assert_ne!(value.compare_to_real(lo).unwrap(), Ordering::Less);
            assert_ne!(value.compare_to_real(hi).unwrap(), Ordering::Greater);
        }
    }
    let text = angular_svg(path);
    assert_eq!(
        export_svg_document(&import_svg_document(&text).unwrap()).unwrap(),
        text
    );
}

#[test]
fn retained_point_bounds_retry_after_actual_exhaustion_without_cached_failure() {
    for _ in 0..2 {
        let path = rational_point_line_across_coarse_pole();
        let query = || path.bounds().cloned();
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
                let result = query();
                assert!(matches!(
                    &result,
                    Err(hypercurve::ExactCurveError::Blocked(blocker))
                        if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted
                ));
                result
            });
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            query,
        );
        let bounds = retry.result.unwrap().unwrap();
        assert_eq!(retry.usage.promotions, 0);
        let warm =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, query);
        assert_eq!(warm.result.unwrap().unwrap(), bounds);
        assert_eq!((warm.usage.refinements, warm.usage.promotions), (0, 0));
    }
}

fn independently_rebuilt_equal_point_pair() -> (CurvePoint2, CurvePoint2) {
    let first = center()
        .rational_image(&RationalMap2::new(
            [polynomial(&[&[0, 1], &[1]]), polynomial(&[&[0, -1], &[1]])],
            polynomial(&[&[1]]),
        ))
        .unwrap();
    let second = translate(
        &translate(&first, Real::one(), -Real::one()),
        -Real::one(),
        Real::one(),
    );
    (first, second)
}

#[test]
fn shared_field_linear_identity_zero_work_control() {
    let (first, second) = independently_rebuilt_equal_point_pair();
    for (a, b) in [(Real::one(), Real::zero()), (Real::zero(), Real::one())] {
        let result =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
                first
                    .affine_form(a.clone(), b.clone(), Real::zero())
                    .compare(&second.affine_form(a, b, Real::zero()))
            });
        assert_eq!(result.result.unwrap().unwrap(), Ordering::Equal);
        assert_eq!((result.usage.refinements, result.usage.promotions), (0, 0));
    }
}

#[test]
fn shared_field_coordinate_order_avoids_independent_projection() {
    let (first, second) = independently_rebuilt_equal_point_pair();
    for axis in [hypercurve::Axis2::X, hypercurve::Axis2::Y] {
        let result =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
                first.compare_coordinate(&second, axis)
            });
        assert_eq!(result.result.unwrap().unwrap(), Ordering::Equal);
        assert_eq!((result.usage.refinements, result.usage.promotions), (0, 0));
    }
    let mut epsilon = Real::one();
    for _ in 0..80 {
        epsilon = (epsilon / Real::from(2)).unwrap();
    }
    let near = translate(&second, epsilon.clone(), -epsilon);
    for (axis, expected) in [
        (hypercurve::Axis2::X, Ordering::Less),
        (hypercurve::Axis2::Y, Ordering::Greater),
    ] {
        let result =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
                first.compare_coordinate(&near, axis)
            });
        assert_eq!(result.result.unwrap().unwrap(), expected);
        assert_eq!((result.usage.refinements, result.usage.promotions), (0, 0));
    }
}

#[test]
fn coordinate_order_keeps_distinct_selected_fiber_fallback() {
    let first = center();
    let CurvePointDefinition2::PolynomialFiber {
        parameter, root, ..
    } = first.definition().unwrap()
    else {
        panic!("original selected fiber");
    };
    let second = CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[-3, 0, 0, 0, 0, 1], &[1]]),
        parameter,
        root,
    }
    .into_point()
    .unwrap();
    // The same beta is shared; gamma_2^5-gamma_1^5=1 and both are positive.
    // These are distinct fibers, so a common-point source cannot be assumed.
    for (a, b) in [
        (first.clone(), second.clone()),
        (
            translate(&first, Real::one(), -Real::one()),
            translate(&second, Real::one(), -Real::one()),
        ),
    ] {
        assert_eq!(
            a.compare_coordinate(&b, hypercurve::Axis2::X).unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            a.compare_coordinate(&b, hypercurve::Axis2::Y).unwrap(),
            Ordering::Less
        );
        assert_eq!(
            b.compare_coordinate(&a, hypercurve::Axis2::Y).unwrap(),
            Ordering::Greater
        );
        assert!(!a.coincides_with(&b).unwrap());
    }
}

#[test]
fn selected_circle_cuts_enter_region_and_repeated_round_offsets() {
    use hypercurve::{BooleanOp, CurvePath2, FillRule, OffsetCornerStyle2, RegionPointLocation};
    let path = circle_path();
    let point = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let curve = &path.curves()[0];
    let (first, second) = curve.split_at(angular_location(curve, &point)).unwrap();
    let complement = circle_path_in_frame(1, false);
    let full = CurvePath2::try_new(vec![
        first,
        second,
        complement.curves()[0].reversed().unwrap(),
    ])
    .unwrap();
    let text = angular_svg(full);
    let replay = import_svg_document(&text).unwrap();
    eprintln!("exact split, complement, and replay completed");
    let region = CurveRegion2::try_from_boundary_paths(replay.paths(), FillRule::NonZero).unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (1, 0));
    assert_eq!(
        region.classify_point(&center()).unwrap(),
        RegionPointLocation::Inside
    );
    assert_eq!(
        region.classify_point(&point).unwrap(),
        RegionPointLocation::Boundary
    );
    eprintln!("region admission and independent interior/boundary completed");
    let union = region
        .boolean_region(&CurveRegion2::empty(), BooleanOp::Union)
        .unwrap();
    let grown = union.offset(q(1, 4), &OffsetCornerStyle2::Round).unwrap();
    assert_eq!(
        grown
            .classify_point(&translate(&center(), q(3, 4), 1.into()))
            .unwrap(),
        RegionPointLocation::Boundary
    );
    let restored = grown.offset(q(-1, 4), &OffsetCornerStyle2::Round).unwrap();
    assert!(
        restored
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        restored.classify_point(&point).unwrap(),
        RegionPointLocation::Boundary
    );
}

fn selected_full_circle(radius: i32, clockwise: bool) -> hypercurve::CurvePath2 {
    let first = circle_path_in_frame(radius, clockwise);
    let second = circle_path_in_frame(radius, !clockwise);
    hypercurve::CurvePath2::try_new(vec![
        first.curves()[0].clone(),
        second.curves()[0].reversed().unwrap(),
    ])
    .unwrap()
}

#[test]
fn selected_circle_regions_preserve_winding_multiplicity_and_reversal() {
    use hypercurve::{BooleanOp, FillRule, RegionPointLocation};
    let path = selected_full_circle(1, true);
    let reverse = path.reversed().unwrap();
    let region =
        CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), FillRule::NonZero)
            .unwrap();
    let reversed =
        CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&reverse), FillRule::NonZero)
            .unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (1, 0));
    assert!(
        region
            .boolean_region(&reversed, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    for rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let doubled =
            CurveRegion2::try_from_boundary_paths(&[path.clone(), path.clone()], rule).unwrap();
        assert_eq!(doubled.is_empty(), rule == FillRule::EvenOdd);
        let canceled =
            CurveRegion2::try_from_boundary_paths(&[path.clone(), reverse.clone()], rule).unwrap();
        assert!(canceled.is_empty());
    }
    for (x, y, location) in [
        (q(0, 1), q(0, 1), RegionPointLocation::Inside),
        (q(3, 5), q(4, 5), RegionPointLocation::Boundary),
        (q(6, 5), q(8, 5), RegionPointLocation::Outside),
    ] {
        assert_eq!(
            region.classify_point(&translate(&center(), x, y)).unwrap(),
            location
        );
    }
}

#[test]
fn selected_circle_regions_preserve_nested_hole_ownership() {
    use hypercurve::{FillRule, RegionPointLocation};
    let outer = selected_full_circle(2, true);
    let inner = selected_full_circle(1, false);
    let annulus =
        CurveRegion2::try_from_boundary_paths(&[outer, inner], FillRule::NonZero).unwrap();
    assert_eq!(annulus.loop_role_counts().unwrap(), (1, 1));
    for (x, location) in [
        (q(0, 1), RegionPointLocation::Outside),
        (q(1, 1), RegionPointLocation::Boundary),
        (q(3, 2), RegionPointLocation::Inside),
        (q(2, 1), RegionPointLocation::Boundary),
        (q(3, 1), RegionPointLocation::Outside),
    ] {
        assert_eq!(
            annulus
                .classify_point(&translate(&center(), x, Real::zero()))
                .unwrap(),
            location
        );
    }
}

#[test]
fn selected_circle_regions_preserve_disconnected_components() {
    use hypercurve::{FillRule, RegionPointLocation, Similarity2};
    let first = selected_full_circle(1, true);
    let transform = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        5.into(),
        0.into(),
    )
    .unwrap();
    let second = first.transform_similarity(&transform).unwrap();
    let region =
        CurveRegion2::try_from_boundary_paths(&[first, second], FillRule::NonZero).unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (2, 0));
    for (x, location) in [
        (q(0, 1), RegionPointLocation::Inside),
        (q(1, 1), RegionPointLocation::Boundary),
        (q(5, 2), RegionPointLocation::Outside),
        (q(4, 1), RegionPointLocation::Boundary),
        (q(5, 1), RegionPointLocation::Inside),
    ] {
        assert_eq!(
            region
                .classify_point(&translate(&center(), x, Real::zero()))
                .unwrap(),
            location
        );
    }
}

#[test]
fn selected_circle_regions_consume_mixed_line_and_circle_boundaries() {
    use hypercurve::{Curve2, CurvePath2, FillRule, RegionPointLocation};
    let half = circle_path();
    let curve = half.curves()[0].clone();
    let diameter = Curve2::try_line(curve.end(), curve.start()).unwrap();
    let path = CurvePath2::try_new(vec![curve, diameter]).unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    assert_eq!(region.loop_role_counts().unwrap(), (1, 0));
    // This chart traverses clockwise from N=(-4/5,3/5) through T=(3/5,4/5).
    for (x, y, location) in [
        (q(3, 10), q(2, 5), RegionPointLocation::Inside),
        (q(-3, 10), q(-2, 5), RegionPointLocation::Outside),
        (q(0, 1), q(0, 1), RegionPointLocation::Boundary),
        (q(3, 5), q(4, 5), RegionPointLocation::Boundary),
    ] {
        assert_eq!(
            region.classify_point(&translate(&center(), x, y)).unwrap(),
            location
        );
    }
}

#[test]
fn selected_circle_diameter_retains_both_endpoint_contacts() {
    let curve = circle_path().curves()[0].clone();
    let diameter = hypercurve::Curve2::try_line(curve.end(), curve.start()).unwrap();
    let result = curve.intersect_curve(&diameter).unwrap();
    assert!(result.blockers().is_empty(), "{:?}", result.blockers());
    assert_eq!(result.contacts().len(), 2);
    assert!(result.overlaps().is_empty());
}

#[test]
fn selected_circle_endpoint_secants_preserve_half_domains_and_cross_signs() {
    use hypercurve::{Curve2, RealSign};
    let curve = circle_path().curves()[0].clone();
    let other = circle_path_in_frame(1, false).curves()[0].clone();
    let a = translate(&center(), q(-7, 25), q(24, 25));
    let b = translate(&center(), Real::one(), Real::zero());
    let c = translate(&center(), -Real::one(), Real::zero());
    let d = translate(&center(), q(7, 25), q(-24, 25));
    for (circle, u, point) in [
        (&curve, q(1, 4), &a),
        (&curve, q(3, 4), &b),
        (&other, q(1, 4), &c),
        (&other, q(3, 4), &d),
    ] {
        assert!(
            circle
                .point_at(&u.into())
                .unwrap()
                .coincides_with(point)
                .unwrap()
        );
    }
    for (start, end, expected) in [(&a, &b, 2), (&c, &d, 0), (&a, &c, 1)] {
        let line = Curve2::try_line(start.clone(), end.clone()).unwrap();
        for reverse_circle in [false, true] {
            let circle = if reverse_circle {
                curve.reversed().unwrap()
            } else {
                curve.clone()
            };
            for reverse_line in [false, true] {
                let chord = if reverse_line {
                    line.reversed().unwrap()
                } else {
                    line.clone()
                };
                let result = circle.intersect_curve(&chord).unwrap();
                assert!(result.blockers().is_empty(), "{:?}", result.blockers());
                assert!(result.overlaps().is_empty());
                assert_eq!(result.contacts().len(), expected);
                for contact in result.contacts() {
                    let at_a = contact.point().coincides_with(&a).unwrap();
                    assert!(at_a || contact.point().coincides_with(&b).unwrap());
                    assert!(contact.is_certified_transverse());
                    let positive = (!at_a) ^ reverse_circle ^ reverse_line;
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(if positive {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    );
                    assert!(
                        circle
                            .point_at(&contact.first().parameter().unwrap())
                            .unwrap()
                            .coincides_with(contact.point())
                            .unwrap()
                    );
                    assert!(
                        chord
                            .point_at(&contact.second().parameter().unwrap())
                            .unwrap()
                            .coincides_with(contact.point())
                            .unwrap()
                    );
                }
            }
        }
    }
}

#[test]
fn selected_circle_region_queries_preserve_policy_certainty() {
    use hypercurve::{CurveCertainty, FillRule, PredicatePolicy, RegionPointLocation};
    let path = selected_full_circle(1, true);
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let region = CurveRegion2::try_from_boundary_paths(
                std::slice::from_ref(&path),
                FillRule::NonZero,
            )
            .unwrap();
            assert_eq!(
                region.classify_point(&center()).unwrap(),
                RegionPointLocation::Inside
            );
            assert_eq!(
                region
                    .classify_point(&translate(&center(), Real::one(), Real::zero()))
                    .unwrap(),
                RegionPointLocation::Boundary
            );
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    }
}

#[test]
fn selected_circle_region_replays_in_a_fresh_process_then_offsets() {
    use hypercurve::{BooleanOp, FillRule, OffsetCornerStyle2, RegionPointLocation};
    const KEY: &str = "HYPERCURVE_SELECTED_REGION_FRESH_REPLAY";
    if let Some(path) = std::env::var_os(KEY) {
        let text = std::fs::read_to_string(path).unwrap();
        let doc = import_svg_document(&text).unwrap();
        let region = doc.region();
        assert_eq!(region.loop_role_counts().unwrap(), (1, 1));
        assert_eq!(
            region.classify_point(&center()).unwrap(),
            RegionPointLocation::Outside
        );
        assert_eq!(
            region
                .classify_point(&translate(&center(), q(3, 2), Real::zero()))
                .unwrap(),
            RegionPointLocation::Inside
        );
        assert_eq!(export_svg_document(&doc).unwrap(), text);
        let grown = region.offset(q(1, 4), &OffsetCornerStyle2::Round).unwrap();
        for x in [q(3, 4), q(9, 4)] {
            assert_eq!(
                grown
                    .classify_point(&translate(&center(), x, Real::zero()))
                    .unwrap(),
                RegionPointLocation::Boundary
            );
        }
        let restored = grown.offset(q(-1, 4), &OffsetCornerStyle2::Round).unwrap();
        assert!(
            restored
                .boolean_region(region, BooleanOp::Xor)
                .unwrap()
                .is_empty()
        );
        let empty = restored
            .boolean_region(region, BooleanOp::Difference)
            .unwrap();
        assert!(empty.is_empty());
        return;
    }
    let region = CurveRegion2::try_from_boundary_paths(
        &[
            selected_full_circle(2, true),
            selected_full_circle(1, false),
        ],
        FillRule::NonZero,
    )
    .unwrap();
    let rebuilt =
        CurveRegion2::try_from_boundary_paths(&region.boundary_paths().unwrap(), FillRule::NonZero)
            .unwrap();
    assert!(
        rebuilt
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let text = export_svg_document(&SvgGeometry2::new(rebuilt, Vec::new(), Vec::new())).unwrap();
    let path = std::env::temp_dir().join(format!(
        "hypercurve-selected-region-replay-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "selected_circle_region_replays_in_a_fresh_process_then_offsets",
            "--exact",
            "--nocapture",
        ])
        .env(KEY, &path)
        .output()
        .unwrap();
    std::fs::remove_file(&path).unwrap();
    assert!(
        output.status.success(),
        "child {:?}: {}\n{}",
        output.status,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn selected_region_replay_depth_limit_rejects_then_retries_exactly() {
    use hypercurve::{
        BooleanOp, FillRule, SvgError, SvgOptions, export_svg_document_with_options,
        import_svg_document_with_options,
    };
    let region = CurveRegion2::try_from_boundary_paths(
        &[
            selected_full_circle(2, true),
            selected_full_circle(1, false),
        ],
        FillRule::NonZero,
    )
    .unwrap();
    let source = SvgGeometry2::new(region.clone(), vec![], vec![]);
    let text = export_svg_document(&source).unwrap();
    let options = SvgOptions {
        max_point_definition_depth: 0,
        ..SvgOptions::default()
    };
    assert!(matches!(
        export_svg_document_with_options(&source, options),
        Err(SvgError::SizeOverflow { .. })
    ));
    assert!(matches!(
        import_svg_document_with_options(&text, options),
        Err(SvgError::SizeOverflow { .. })
    ));
    let replay = import_svg_document(&text).unwrap();
    assert!(
        replay
            .region()
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert_eq!(export_svg_document(&replay).unwrap(), text);
}

#[test]
fn selected_circle_regions_boolean_annulus_and_empty_offsets() {
    use hypercurve::{BooleanOp, FillRule, OffsetCornerStyle2, RegionPointLocation};
    let disk = |radius| {
        CurveRegion2::try_from_boundary_paths(
            &[selected_full_circle(radius, true)],
            FillRule::NonZero,
        )
        .unwrap()
    };
    let outer = disk(2);
    let inner = disk(1);
    let annulus = outer.boolean_region(&inner, BooleanOp::Difference).unwrap();
    assert_eq!(annulus.loop_role_counts().unwrap(), (1, 1));
    for (x, location) in [
        (q(0, 1), RegionPointLocation::Outside),
        (q(1, 1), RegionPointLocation::Boundary),
        (q(3, 2), RegionPointLocation::Inside),
        (q(2, 1), RegionPointLocation::Boundary),
        (q(3, 1), RegionPointLocation::Outside),
    ] {
        assert_eq!(
            annulus
                .classify_point(&translate(&center(), x, Real::zero()))
                .unwrap(),
            location
        );
    }
    let xor = outer.boolean_region(&inner, BooleanOp::Xor).unwrap();
    assert!(
        xor.boolean_region(&annulus, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert!(
        annulus
            .boolean_region(&inner, BooleanOp::Intersection)
            .unwrap()
            .is_empty()
    );
    let union = annulus.boolean_region(&inner, BooleanOp::Union).unwrap();
    assert!(
        union
            .boolean_region(&outer, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let erased = inner
        .offset((-2).into(), &OffsetCornerStyle2::Round)
        .unwrap();
    assert!(erased.is_empty());
    assert!(
        erased
            .offset(q(1, 4), &OffsetCornerStyle2::Round)
            .unwrap()
            .is_empty()
    );
    assert!(
        erased
            .boolean_region(&inner, BooleanOp::Union)
            .unwrap()
            .boolean_region(&inner, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

fn assert_circle_chord_contact_parameters(
    circle: &hypercurve::Curve2,
    chord: &hypercurve::Curve2,
    contact: &hypercurve::CurveIntersectionContact2,
) {
    assert!(
        circle
            .point_at(&contact.first().parameter().unwrap())
            .unwrap()
            .coincides_with(contact.point())
            .unwrap()
    );
    assert!(
        chord
            .point_at(&contact.second().parameter().unwrap())
            .unwrap()
            .coincides_with(contact.point())
            .unwrap()
    );
    assert!(
        contact
            .point()
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(contact.point())
            .unwrap()
    );
}

#[test]
fn selected_circle_one_incident_radial_chords_keep_signed_radius_and_traversal() {
    use hypercurve::{Curve2, RealSign};
    let n = translate(&center(), q(-4, 5), q(3, 5));
    let minus_n = translate(&center(), q(4, 5), q(-3, 5));
    let far = translate(&center(), q(-12, 5), q(9, 5));
    for radius in [-1, 1] {
        for clockwise in [false, true] {
            let circle = circle_path_in_frame(radius, clockwise).curves()[0].clone();
            for reverse_line in [false, true] {
                let chord = if reverse_line {
                    Curve2::try_line(minus_n.clone(), far.clone()).unwrap()
                } else {
                    Curve2::try_line(far.clone(), minus_n.clone()).unwrap()
                };
                let result = circle.intersect_curve(&chord).unwrap();
                assert!(result.blockers().is_empty(), "{:?}", result.blockers());
                assert!(result.overlaps().is_empty());
                assert_eq!(result.contacts().len(), 2);
                for contact in result.contacts() {
                    let at_n = contact.point().coincides_with(&n).unwrap();
                    assert!(at_n || contact.point().coincides_with(&minus_n).unwrap());
                    let positive = (!at_n) ^ (!clockwise) ^ reverse_line;
                    assert!(contact.is_certified_transverse());
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(if positive {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    );
                    assert_circle_chord_contact_parameters(&circle, &chord, contact);
                }
            }
        }
    }
}

#[test]
fn selected_circle_one_incident_chords_clip_the_second_root_and_keep_tangency() {
    use hypercurve::{Curve2, RealSign};
    let circle = circle_path().curves()[0].clone();
    let n = translate(&center(), q(-4, 5), q(3, 5));
    // The opposite radial contact lies outside each finite chord. The last
    // chord is tangent because N dot T=0 for T=(3/5,4/5).
    for (other, tangent) in [
        (translate(&center(), q(-12, 5), q(9, 5)), false),
        (center(), false),
        (translate(&center(), q(-1, 5), q(7, 5)), true),
    ] {
        for reverse in [false, true] {
            let chord = if reverse {
                Curve2::try_line(other.clone(), n.clone()).unwrap()
            } else {
                Curve2::try_line(n.clone(), other.clone()).unwrap()
            };
            let result = circle.intersect_curve(&chord).unwrap();
            assert!(result.blockers().is_empty(), "{:?}", result.blockers());
            assert!(result.overlaps().is_empty());
            assert_eq!(result.contacts().len(), 1);
            let contact = &result.contacts()[0];
            assert!(contact.point().coincides_with(&n).unwrap());
            assert_eq!(contact.is_certified_transverse(), !tangent);
            assert_eq!(
                contact.tangent_cross_sign() == Some(RealSign::Zero),
                tangent
            );
            assert_circle_chord_contact_parameters(&circle, &chord, contact);
        }
    }
}

#[test]
fn selected_circle_one_incident_oblique_chord_retains_both_exact_contacts() {
    use hypercurve::{Curve2, RealSign};
    let circle = circle_path().curves()[0].clone();
    let a = translate(&center(), q(-7, 25), q(24, 25));
    let second = translate(&center(), Real::one(), Real::zero());
    // B=A+2(Q-A); A and Q are independently specified unit-circle points.
    let b = translate(&center(), q(57, 25), q(-24, 25));
    for reverse_circle in [false, true] {
        let circle = if reverse_circle {
            circle.reversed().unwrap()
        } else {
            circle.clone()
        };
        for reverse_line in [false, true] {
            let chord = if reverse_line {
                Curve2::try_line(b.clone(), a.clone()).unwrap()
            } else {
                Curve2::try_line(a.clone(), b.clone()).unwrap()
            };
            let result = circle.intersect_curve(&chord).unwrap();
            assert!(result.blockers().is_empty(), "{:?}", result.blockers());
            assert!(result.overlaps().is_empty());
            assert_eq!(result.contacts().len(), 2);
            for contact in result.contacts() {
                let at_a = contact.point().coincides_with(&a).unwrap();
                assert!(at_a || contact.point().coincides_with(&second).unwrap());
                assert!(contact.is_certified_transverse());
                let positive = (!at_a) ^ reverse_circle ^ reverse_line;
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if positive {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    })
                );
                assert_circle_chord_contact_parameters(&circle, &chord, contact);
            }
        }
    }
}

#[test]
fn selected_circle_one_incident_fiber_chord_keeps_correlated_second_contact() {
    use hypercurve::{Curve2, RealSign};
    let a = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let b = algebraic_angular_point(-2, true, Real::one(), Real::zero());
    let opposite = algebraic_angular_point(-1, true, Real::one(), Real::zero());
    let line = Curve2::try_line(a.clone(), b).unwrap();
    // Independently constructed antipodal points lie on the two selected
    // semicircles. Each half keeps exactly its own contact.
    for clockwise in [false, true] {
        let circle = circle_path_in_frame(1, clockwise).curves()[0].clone();
        let expected = if clockwise { &a } else { &opposite };
        let result = circle.intersect_curve(&line).unwrap();
        assert!(result.blockers().is_empty(), "{:?}", result.blockers());
        assert!(result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 1);
        let contact = &result.contacts()[0];
        assert!(contact.point().coincides_with(expected).unwrap());
        assert!(contact.is_certified_transverse());
        assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Negative));
        assert_circle_chord_contact_parameters(&circle, &line, contact);
        let parameter = contact.first().parameter().unwrap();
        let (first, second) = circle.split_at(parameter).unwrap();
        let saved = angular_svg(hypercurve::CurvePath2::try_new(vec![first, second]).unwrap());
        let replay = import_svg_document(&saved).unwrap();
        assert!(
            replay.paths()[0].curves()[0]
                .end()
                .coincides_with(expected)
                .unwrap()
        );
        assert_eq!(export_svg_document(&replay).unwrap(), saved);
    }
}

#[test]
fn selected_field_chords_retain_partial_overlap_through_similarity_and_reversal() {
    use hypercurve::{Curve2, CurveOverlapOrientation2, Similarity2};
    let radial = |scale: i32| translate(&center(), q(-4 * scale, 5), q(3 * scale, 5));
    let first = Curve2::try_line(radial(3), radial(-1)).unwrap();
    let second = Curve2::try_line(radial(1), radial(-3)).unwrap();
    for shift in [0, 3] {
        let transform = Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            shift.into(),
            (-shift).into(),
        )
        .unwrap();
        let expected =
            [radial(1), radial(-1)].map(|point| translate(&point, shift.into(), (-shift).into()));
        for reverse_first in [false, true] {
            let first = first.transform_similarity(&transform).unwrap();
            let first = if reverse_first {
                first.reversed().unwrap()
            } else {
                first
            };
            for reverse_second in [false, true] {
                let second = second.transform_similarity(&transform).unwrap();
                let second = if reverse_second {
                    second.reversed().unwrap()
                } else {
                    second
                };
                for (a, b) in [(&first, &second), (&second, &first)] {
                    let result = a.intersect_curve(b).unwrap();
                    assert!(result.blockers().is_empty(), "{:?}", result.blockers());
                    assert!(result.contacts().is_empty());
                    assert_eq!(result.overlaps().len(), 1);
                    let overlap = &result.overlaps()[0];
                    assert_eq!(
                        overlap.orientation(),
                        if reverse_first ^ reverse_second {
                            CurveOverlapOrientation2::Reversed
                        } else {
                            CurveOverlapOrientation2::Same
                        }
                    );
                    assert!(overlap.includes_start() && overlap.includes_end());
                    let endpoints = [
                        (
                            overlap.first_range().start(),
                            overlap.second_range().start(),
                        ),
                        (overlap.first_range().end(), overlap.second_range().end()),
                    ];
                    let mut found = [false; 2];
                    for (pa, pb) in endpoints {
                        let point = a.point_at(pa).unwrap();
                        assert!(point.coincides_with(&b.point_at(pb).unwrap()).unwrap());
                        let index = expected
                            .iter()
                            .position(|p| point.coincides_with(p).unwrap())
                            .unwrap();
                        assert!(!found[index]);
                        found[index] = true;
                    }
                    assert_eq!(found, [true, true]);
                }
            }
        }
    }
}

#[test]
fn selected_field_chord_sides_distinguish_near_parallel_and_transverse_supports() {
    use hypercurve::{Curve2, RealSign};
    let radial = |scale: i32| translate(&center(), q(-4 * scale, 5), q(3 * scale, 5));
    let first = Curve2::try_line(radial(3), radial(-1)).unwrap();
    let tiny = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let shifted = |point: CurvePoint2| translate(&point, &tiny * q(3, 5), &tiny * q(4, 5));
    let near = Curve2::try_line(shifted(radial(1)), shifted(radial(-3))).unwrap();
    let separate = Curve2::try_line(radial(4), radial(5)).unwrap();
    for chord in [near, separate] {
        let result = first.intersect_curve(&chord).unwrap();
        assert!(result.blockers().is_empty(), "{:?}", result.blockers());
        assert!(result.contacts().is_empty());
        assert!(result.overlaps().is_empty());
    }
    let transverse = Curve2::try_line(
        translate(&center(), q(-3, 5), q(-4, 5)),
        translate(&center(), q(3, 5), q(4, 5)),
    )
    .unwrap();
    let result = first.intersect_curve(&transverse).unwrap();
    assert!(result.blockers().is_empty(), "{:?}", result.blockers());
    assert!(result.overlaps().is_empty());
    assert_eq!(result.contacts().len(), 1);
    let contact = &result.contacts()[0];
    assert!(contact.point().coincides_with(&center()).unwrap());
    assert!(center().coincides_with(contact.point()).unwrap());
    let nearby = translate(&center(), &tiny * q(3, 5), &tiny * q(4, 5));
    assert!(!contact.point().coincides_with(&nearby).unwrap());
    assert!(!nearby.coincides_with(contact.point()).unwrap());
    assert!(contact.is_certified_transverse());
    assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Positive));
    assert_circle_chord_contact_parameters(&first, &transverse, contact);
}

#[test]
fn selected_circle_one_incident_contacts_retry_and_preserve_policy_receipts() {
    use hypercurve::{Curve2, CurveCertainty, PredicatePolicy};
    let circle = circle_path_in_frame(1, false).curves()[0].clone();
    let a = algebraic_angular_point(1, true, Real::one(), Real::zero());
    let b = algebraic_angular_point(-2, true, Real::one(), Real::zero());
    let opposite = algebraic_angular_point(-1, true, Real::one(), Real::zero());
    let line = Curve2::try_line(a, b).unwrap();
    let query = || circle.intersect_curve(&line);
    for _ in 0..2 {
        let denied =
            hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, query);
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            query,
        );
        let result = retry.result.unwrap().unwrap();
        assert!(result.blockers().is_empty(), "{:?}", result.blockers());
        assert_eq!(result.contacts().len(), 1);
        assert!(
            result.contacts()[0]
                .point()
                .coincides_with(&opposite)
                .unwrap()
        );
        assert_eq!(retry.usage.promotions, 0);
    }
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let result = query().unwrap();
            assert!(result.blockers().is_empty(), "{:?}", result.blockers());
            assert_eq!(result.contacts().len(), 1);
            let contact = &result.contacts()[0];
            assert!(contact.point().coincides_with(&opposite).unwrap());
            assert_circle_chord_contact_parameters(&circle, &line, contact);
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    }
}
