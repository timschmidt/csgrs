#![cfg(feature = "svg")]

mod support;

use hypercurve::{
    BooleanOp, CubicBezier2, Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, Point2,
    PredicatePolicy, QuadraticBezier2, Real, Similarity2, SvgError, SvgGeometry2,
    export_svg_document, import_svg_document, parse_svg_path_data,
};

fn ratio(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn svg_path_decimal_coordinates_remain_exact() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let result =
            support::under(&policy, || parse_svg_path_data("M.1 -.2 l.2 .3 H.4 v.1")).unwrap();
        assert_eq!(result.certainty, CurveCertainty::Certified);
        let curves = result.value[0].path().curves();
        let expected = [
            Point2::new(ratio(1, 10), ratio(-1, 5)),
            Point2::new(ratio(3, 10), ratio(1, 10)),
            Point2::new(ratio(2, 5), ratio(1, 10)),
            Point2::new(ratio(2, 5), ratio(1, 5)),
        ];
        assert_eq!(curves.len(), 3);
        for (curve, ends) in curves.iter().zip(expected.windows(2)) {
            for (parameter, expected) in [(Real::zero(), &ends[0]), (Real::one(), &ends[1])] {
                let actual = curve.point_at(&parameter.into()).unwrap();
                assert_eq!(actual.coordinates().unwrap(), expected);
            }
        }
    }
}

#[test]
fn svg_path_scientific_tokens_and_distinct_coordinates_are_not_rounded() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (data, start, end) in [
            (
                "M+1.e-1-.2e+0.3e0-.4e0",
                Point2::new(ratio(1, 10), ratio(-1, 5)),
                Point2::new(ratio(3, 10), ratio(-2, 5)),
            ),
            (
                "M9007199254740992 0L9007199254740993 0",
                Point2::new(Real::from(9_007_199_254_740_992_i64), Real::zero()),
                Point2::new(Real::from(9_007_199_254_740_993_i64), Real::zero()),
            ),
            (
                "M0 0L1e-400 0",
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(
                    (Real::one() / Real::from(10).powi(400.into()).unwrap()).unwrap(),
                    Real::zero(),
                ),
            ),
        ] {
            let result = support::under(&policy, || parse_svg_path_data(data)).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let curves = result.value[0].path().curves();
            assert_eq!(curves.len(), 1);
            for (parameter, expected) in [(Real::zero(), start), (Real::one(), end)] {
                let actual = curves[0].point_at(&parameter.into()).unwrap();
                assert_eq!(actual.coordinates().unwrap(), &expected);
            }
        }
    }
}

#[test]
fn svg_bezier_controls_and_reflections_retain_decimal_values() {
    let p = |x, y| Point2::new(ratio(x, 10), ratio(y, 10));
    let expected = [
        Curve2::from(CubicBezier2::new(p(1, 2), p(3, 4), p(5, 6), p(7, 8))),
        Curve2::from(CubicBezier2::new(p(7, 8), p(9, 10), p(9, 10), p(11, 12))),
        Curve2::from(QuadraticBezier2::new(p(11, 12), p(13, 14), p(15, 16))),
        Curve2::from(QuadraticBezier2::new(p(15, 16), p(17, 18), p(17, 18))),
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for data in [
            "M.1 .2C.3 .4 .5 .6 .7 .8S.9 1 1.1 1.2Q1.3 1.4 1.5 1.6T1.7 1.8",
            "m.1 .2c.2 .2 .4 .4 .6 .6s.2 .2 .4 .4q.2 .2 .4 .4t.2 .2",
        ] {
            let result = support::under(&policy, || parse_svg_path_data(data)).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value[0].path().curves(), expected);
        }
    }
}

#[test]
fn svg_compact_arc_flags_preserve_exact_radii_and_ellipse_rejection() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for data in [
            "M.1 0A.1 .1 0 01-.1 0",
            "M.1 0A-.1 -1e-1 0 10-.1 0",
            "M-.1 0a.1 .1 0 00.2 0",
            "M-.1 0a-.1 -.1 0 11.2 0",
        ] {
            let result = support::under(&policy, || parse_svg_path_data(data)).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let curve = &result.value[0].path().curves()[0];
            let midpoint = curve.point_at(&ratio(1, 2).into()).unwrap();
            assert_eq!(midpoint.coordinates().unwrap().x(), &Real::zero());
            assert_eq!(midpoint.coordinates().unwrap().y().abs(), ratio(1, 10));
        }
        assert!(matches!(
            support::under(&policy, || {
                parse_svg_path_data("M.1 0A.1 .10000000000000001 0 01-.1 0")
            }),
            Err(SvgError::Unsupported(_))
        ));
        for malformed in [
            "M0 0A1 1 0 2 0 1 0",
            "M0 0L1e 2",
            "M0 0LNaN 1",
            "M0 0L1 2em",
            "M0 0L1 2ex",
            "M0 0L1 2Em",
            "M0 0L1 2Ex",
        ] {
            assert!(matches!(
                parse_svg_path_data(malformed),
                Err(SvgError::MalformedInput(_))
            ));
        }
    }
}

#[test]
fn svg_decimal_filled_region_survives_boolean_and_native_replay() {
    const REPLAY: &str = "HYPERCURVE_DECIMAL_SVG_REPLAY";
    let replay_input = std::env::var(REPLAY).ok();
    let points = [
        Point2::new(ratio(1, 10), ratio(1, 5)),
        Point2::new(ratio(2, 5), ratio(1, 5)),
        Point2::new(ratio(2, 5), ratio(3, 5)),
        Point2::new(ratio(1, 10), ratio(3, 5)),
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let result =
            support::under(&policy, || {
                let boundary = CurvePath2::try_new(
                    (0..4)
                        .map(|i| {
                            Curve2::try_line(
                                points[i].clone().into(),
                                points[(i + 1) % 4].clone().into(),
                            )
                            .unwrap()
                        })
                        .collect(),
                )
                .unwrap();
                let expected =
                    CurveRegion2::try_from_boundary_paths(&[boundary], FillRule::NonZero).unwrap();
                let imported = import_svg_document(replay_input.as_deref().unwrap_or(
                "<svg xmlns=\"http://www.w3.org/2000/svg\"><path d=\"M.1 .2h.3v.4h-.3z\"/></svg>",
            ))?;
                assert!(
                    imported
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                let text = export_svg_document(&imported)?;
                #[cfg(not(target_arch = "wasm32"))]
                if replay_input.is_none() {
                    let child = std::process::Command::new(std::env::current_exe().unwrap())
                        .args([
                            "--exact",
                            "svg_decimal_filled_region_survives_boolean_and_native_replay",
                            "--nocapture",
                        ])
                        .env(REPLAY, &text)
                        .output()
                        .unwrap();
                    let stdout = String::from_utf8(child.stdout).unwrap();
                    assert!(
                        child.status.success(),
                        "{stdout}\n{}",
                        String::from_utf8_lossy(&child.stderr)
                    );
                    assert!(
                        stdout.lines().any(|line| line
                            .starts_with("test result: ok. 1 passed; 0 failed; 0 ignored;")),
                        "fresh replay must execute its acceptance test: {stdout}"
                    );
                    for line in stdout.lines() {
                        println!("fresh SVG replay: {line}");
                    }
                }
                let replayed = import_svg_document(&text)?;
                let transform = Similarity2::try_from_real_affine(
                    2.into(),
                    0.into(),
                    0.into(),
                    (-2).into(),
                    ratio(1, 7),
                    ratio(-2, 7),
                )
                .unwrap();
                let moved = replayed.region().transform_similarity(&transform).unwrap();
                let moved_expected = expected.transform_similarity(&transform).unwrap();
                assert!(
                    moved
                        .boolean_region(&moved_expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                assert!(
                    replayed
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                let remainder = replayed
                    .region()
                    .boolean_region(&expected, BooleanOp::Difference)
                    .unwrap();
                assert!(remainder.is_empty());
                assert!(
                    remainder
                        .boolean_region(&expected, BooleanOp::Union)
                        .unwrap()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                Ok::<_, SvgError>(SvgGeometry2::new(remainder, vec![], vec![]))
            })
            .unwrap();
        assert_eq!(result.certainty, CurveCertainty::Certified);
    }
}
