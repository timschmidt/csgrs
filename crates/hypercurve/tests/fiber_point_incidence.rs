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
