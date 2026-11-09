#![cfg(feature = "svg")]

#[macro_use]
mod support;

use hypercurve::{
    BooleanOp, Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, Point2, PredicatePolicy,
    Real, Similarity2, SvgError, export_svg_document, import_svg_document,
};

// The bounds supplied by each witness are independent exact expectations.
// This helper only assembles their four native line supports.
fn expected_rectangle([x0, y0, x1, y1]: [Real; 4]) -> CurveRegion2 {
    let points = [
        Point2::new(x0.clone(), y0.clone()),
        Point2::new(x1.clone(), y0),
        Point2::new(x1, y1.clone()),
        Point2::new(x0, y1),
    ];
    let boundary = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::try_line(points[i].clone().into(), points[(i + 1) % 4].clone().into())
                    .unwrap()
            })
            .collect(),
    )
    .unwrap();
    CurveRegion2::try_from_boundary_paths(&[boundary], FillRule::NonZero).unwrap()
}

#[test]
fn svg_unrounded_rectangles_retain_exact_values_and_distinct_extents() {
    let tiny = (Real::one() / Real::from(10).powi(400.into()).unwrap()).unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (attributes, [x0, y0, x1, y1]) in [
            (
                r#"x=".1" y=".2" width=".3" height=".4""#,
                [r!(1 / 10), r!(1 / 5), r!(2 / 5), r!(3 / 5)],
            ),
            (
                r#"x="+1.e-1" y="-.2e0" width="3e-1" height="4e-1""#,
                [r!(1 / 10), r!(-1 / 5), r!(2 / 5), r!(1 / 5)],
            ),
            (
                r#"x="9007199254740992" y="9007199254740992" width="1" height="1""#,
                [
                    r!(9_007_199_254_740_992_i64),
                    r!(9_007_199_254_740_992_i64),
                    r!(9_007_199_254_740_993_i64),
                    r!(9_007_199_254_740_993_i64),
                ],
            ),
            (
                r#"width="1e-400" height="1e-400""#,
                [r!(0), r!(0), tiny.clone(), tiny.clone()],
            ),
        ] {
            let outcome = support::under(&policy, || {
                let expected = expected_rectangle([x0, y0, x1, y1]);
                let imported = import_svg_document(&format!("<svg><rect {attributes}/></svg>"))?;
                assert!(!imported.region().is_empty(), "{attributes}");
                assert!(imported.wires().is_empty() && imported.paths().is_empty());
                assert_eq!(imported.region().loop_role_counts().unwrap(), (1, 0));
                let boundaries = imported.region().boundary_paths().unwrap();
                assert_eq!(boundaries.len(), 1);
                assert_eq!(boundaries[0].curves().len(), 4);
                assert!(
                    boundaries[0]
                        .curves()
                        .iter()
                        .all(|curve| curve.family() == hypercurve::CurveFamily2::Line)
                );
                assert!(
                    imported
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty(),
                    "{attributes}"
                );
                let text = export_svg_document(&imported)?;
                let replayed = import_svg_document(&text)?;
                assert_eq!(export_svg_document(&replayed)?, text);
                assert!(
                    replayed
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty(),
                    "native replay: {attributes}"
                );
                Ok::<_, SvgError>(())
            })
            .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
        }
    }
}

#[test]
fn svg_zero_corner_rectangles_combine_exact_paint_and_transform() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for corners in [
            "",
            r#"rx="0""#,
            r#"ry="-0""#,
            r#"rx="0" ry=".1""#,
            r#"rx=".1" ry="0""#,
            r#"rx="1e-400" ry="0""#,
        ] {
            let outcome = support::under(&policy, || {
                let expected = expected_rectangle([
                    r!(3 / 10), r!(4 / 5), r!(9 / 10), r!(2),
                ]);
                let document = format!(
                    r#"<svg opacity="1e-200"><g opacity="1e-200" transform="translate(.1 .2)"><rect {corners} x=".1" y=".2" width=".3" height=".4" transform="scale(2 3)" fill-opacity="1e-400%"/></g></svg>"#
                );
                let imported = import_svg_document(&document)?;
                assert!(!imported.is_empty(), "{corners}");
                assert!(
                    imported
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty(),
                    "{corners}"
                );
                Ok::<_, SvgError>(())
            })
            .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
        }
    }
}

#[test]
fn svg_rectangle_dimensions_and_corner_guards_remain_typed() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for selection in [
            "",
            r#"fill="none" stroke="none""#,
            r#"opacity="0""#,
            r#"display="none""#,
            r#"visibility="hidden""#,
        ] {
            for attributes in [
                r#"width="0" height="1""#,
                r#"width="-0" height="1""#,
                r#"width="-1e-400" height="1""#,
                r#"width="1" height="-1e-400""#,
                r#"x="NaN" width="1" height="1""#,
                r#"y="inf" width="1" height="1""#,
                r#"width="1e400" height="1""#,
                r#"width="1em" height="1""#,
                r#"width="1/2" height="1""#,
                r#"width="1""#,
                r#"width="1" height="1" rx="-1e-400""#,
                r#"width="1" height="1" ry="-1e-400""#,
                r#"width="1" height="1" rx="NaN""#,
                r#"width="1" height="1" ry="1e400""#,
            ] {
                let document = format!("<svg><rect {selection} {attributes}/></svg>");
                assert!(
                    matches!(
                        support::under(&policy, || import_svg_document(&document)),
                        Err(SvgError::MalformedInput(detail)) if !detail.is_empty()
                    ),
                    "{document}"
                );
            }
        }
    }
}

#[test]
fn svg_rectangle_native_replay_continues_through_transform_and_boolean() {
    const REPLAY: &str = "HYPERCURVE_RECTANGLE_SVG_REPLAY";
    let replay_input = std::env::var(REPLAY).ok();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome = support::under(&policy, || {
            let expected = expected_rectangle([r!(1 / 10), r!(1 / 5), r!(2 / 5), r!(3 / 5)]);
            let authored = r#"<svg><rect x=".1" y=".2" width=".3" height=".4"/></svg>"#;
            let imported = import_svg_document(replay_input.as_deref().unwrap_or(authored))?;
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
                        "svg_rectangle_native_replay_continues_through_transform_and_boolean",
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
                assert!(stdout.lines().any(|line| {
                    line.starts_with("test result: ok. 1 passed; 0 failed; 0 ignored;")
                }));
                for line in stdout.lines() {
                    println!("fresh rectangle replay: {line}");
                }
            }
            let replayed = import_svg_document(&text)?;
            assert_eq!(export_svg_document(&replayed)?, text);
            let reflection = Similarity2::try_from_real_affine(
                r!(2),
                r!(0),
                r!(0),
                r!(-2),
                r!(1 / 7),
                r!(-2 / 7),
            )
            .unwrap();
            assert!(
                replayed
                    .region()
                    .transform_similarity(&reflection)
                    .unwrap()
                    .boolean_region(
                        &expected.transform_similarity(&reflection).unwrap(),
                        BooleanOp::Xor,
                    )
                    .unwrap()
                    .is_empty()
            );
            let empty = replayed
                .region()
                .boolean_region(&expected, BooleanOp::Difference)
                .unwrap();
            assert!(empty.is_empty());
            assert!(
                empty
                    .boolean_region(&expected, BooleanOp::Union)
                    .unwrap()
                    .boolean_region(&expected, BooleanOp::Xor)
                    .unwrap()
                    .is_empty()
            );
            Ok::<_, SvgError>(())
        })
        .unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
    }
}

#[test]
fn svg_valid_hidden_rectangles_remain_empty_and_composable() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for selection in [
            r#"fill="none" stroke="none""#,
            r#"opacity="0""#,
            r#"display="none""#,
            r#"visibility="hidden""#,
        ] {
            let outcome = support::under(&policy, || {
                let document = format!(
                    r#"<svg><rect {selection} x=".1" y=".2" width=".3" height=".4"/></svg>"#
                );
                let imported = import_svg_document(&document)?;
                assert!(imported.is_empty());
                let text = export_svg_document(&imported)?;
                let replayed = import_svg_document(&text)?;
                assert!(replayed.is_empty());
                assert_eq!(export_svg_document(&replayed)?, text);
                let expected = expected_rectangle([r!(0), r!(0), r!(1), r!(1)]);
                assert!(
                    replayed
                        .region()
                        .boolean_region(&expected, BooleanOp::Union)
                        .unwrap()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                Ok::<_, SvgError>(())
            })
            .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
        }
    }
}

#[test]
fn svg_rectangle_construction_does_not_require_finite_float_sums() {
    // Each authored attribute is finite binary64, while their exact sum need
    // not be. This witness covers native construction/Booleans, independently
    // of SVG export's separately declared finite-display projection contract.
    let huge = Real::from(10).powi(308.into()).unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome = support::under(&policy, || {
            let expected = expected_rectangle([huge.clone(), r!(0), r!(2) * &huge, r!(1)]);
            let imported =
                import_svg_document(r#"<svg><rect x="1e308" width="1e308" height="1"/></svg>"#)?;
            assert!(!imported.is_empty());
            assert!(
                imported
                    .region()
                    .boolean_region(&expected, BooleanOp::Xor)
                    .unwrap()
                    .is_empty()
            );
            let erased = imported
                .region()
                .boolean_region(&expected, BooleanOp::Difference)
                .unwrap();
            assert!(erased.is_empty());
            assert!(
                erased
                    .boolean_region(&expected, BooleanOp::Union)
                    .unwrap()
                    .boolean_region(&expected, BooleanOp::Xor)
                    .unwrap()
                    .is_empty()
            );
            Ok::<_, SvgError>(())
        })
        .unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
    }
}

#[test]
fn svg_hidden_rounded_rectangles_do_not_require_sampled_materialization() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for selection in [
            r#"opacity="0""#,
            r#"display="none""#,
            r#"visibility="hidden""#,
            r#"fill="none" stroke="none""#,
        ] {
            for attributes in [
                r#"width="1e-400" height="1e-400" rx="1e-400" ry="1e-400""#,
                r#"x="1e308" width="1e308" height="1" rx="1" ry="1""#,
            ] {
                let outcome = support::under(&policy, || {
                    let imported = import_svg_document(&format!(
                        "<svg><rect {selection} {attributes}/></svg>"
                    ))?;
                    assert!(imported.is_empty(), "{selection} {attributes}");
                    let text = export_svg_document(&imported)?;
                    let replayed = import_svg_document(&text)?;
                    assert!(replayed.is_empty());
                    assert_eq!(export_svg_document(&replayed)?, text);
                    let unit = expected_rectangle([r!(0), r!(0), r!(1), r!(1)]);
                    let recovered = replayed
                        .region()
                        .boolean_region(&unit, BooleanOp::Union)
                        .unwrap();
                    assert!(
                        recovered
                            .boolean_region(&unit, BooleanOp::Xor)
                            .unwrap()
                            .is_empty()
                    );
                    Ok::<_, SvgError>(())
                })
                .unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
            }
        }
    }
}
