#![cfg(feature = "svg")]

#[macro_use]
mod support;

use hypercurve::{
    BooleanOp, Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, Point2, PredicatePolicy,
    Segment2, Similarity2, SvgError, export_svg_document, import_svg_document,
};

fn unit_square() -> CurveRegion2 {
    let points = [pt!(0, 0), pt!(1, 0), pt!(1, 1), pt!(0, 1)];
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
fn svg_positive_paint_factors_do_not_underflow() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for attributes in [
            "opacity=\"1e-400\"",
            "opacity=\"1e-400%\"",
            "fill-opacity=\"1e-400\"",
            "fill-opacity=\"1e-400%\"",
            "style=\"opacity:1e-400;fill-opacity:1e-400%\"",
            // Real retains scientific scales beyond Rational's materialized
            // exponent bound. A tiny positive factor still selects paint.
            "opacity=\"1e-10000000\"",
            "fill-opacity=\"1e-10000000%\"",
            "opacity=\"0\" style=\"opacity:1e-400\"",
        ] {
            let document = format!("<svg><path {attributes} d=\"M0 0h1v1h-1z\"/></svg>");
            let outcome = support::under(&policy, || {
                let imported = import_svg_document(&document)?;
                assert!(!imported.is_empty(), "{attributes}");
                assert!(imported.wires().is_empty());
                assert!(imported.paths().is_empty());
                assert!(
                    imported
                        .region()
                        .boolean_region(&unit_square(), BooleanOp::Xor)
                        .unwrap()
                        .is_empty(),
                    "{attributes}"
                );
                Ok::<_, SvgError>(())
            })
            .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
        }
        for attributes in [
            "stroke-opacity=\"1e-400\"",
            "stroke-opacity=\"1e-400%\"",
            "stroke-width=\"1e-400\"",
            "stroke-width=\"1e-10000000\"",
            "style=\"stroke-opacity:1e-400%;stroke-width:1e-400\"",
        ] {
            let document = format!(
                "<svg><path fill=\"none\" stroke=\"black\" {attributes} d=\"M0 0L1 1\"/></svg>"
            );
            let outcome = support::under(&policy, || {
                let imported = import_svg_document(&document)?;
                assert!(!imported.is_empty(), "{attributes}");
                assert!(imported.region().is_empty());
                assert!(imported.paths().is_empty());
                assert_eq!(imported.wires().len(), 1, "{attributes}");
                let segments = imported.wires()[0].segments();
                assert_eq!(segments.len(), 1);
                let Segment2::Line(line) = &segments[0] else {
                    panic!("a positive paint factor must retain the authored line");
                };
                assert_eq!(line.start(), &pt!(0, 0));
                assert_eq!(line.end(), &pt!(1, 1));
                Ok::<_, SvgError>(())
            })
            .unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
        }
    }
}

#[test]
fn svg_invalid_paint_ranges_remain_typed_errors() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for name in ["opacity", "fill-opacity", "stroke-opacity"] {
            for value in [
                "-1e-400",
                "-1e-400%",
                "-1e-10000000",
                "1.00000000000000000001",
                "100.000000000000000001%",
                "NaN",
                "inf",
                "1e400",
                "1e400%",
                "1/2",
            ] {
                for attribute in [
                    format!("{name}=\"{value}\""),
                    format!("style=\"{name}:{value}\""),
                ] {
                    let document = format!("<svg><path {attribute} d=\"M0 0h1v1h-1z\"/></svg>");
                    let error =
                        support::under(&policy, || import_svg_document(&document)).unwrap_err();
                    assert!(
                        matches!(error, SvgError::MalformedInput(_)),
                        "{attribute}: {error:?}"
                    );
                }
            }
        }
        for value in [
            "-1e-400",
            "-1e-10000000",
            "NaN",
            "inf",
            "1e400",
            "1%",
            "1/2",
        ] {
            let document = format!(
                "<svg><path stroke=\"black\" stroke-width=\"{value}\" d=\"M0 0L1 1\"/></svg>"
            );
            let error = support::under(&policy, || import_svg_document(&document)).unwrap_err();
            assert!(
                matches!(error, SvgError::MalformedInput(_)),
                "{value}: {error:?}"
            );
        }
    }
}

#[test]
fn svg_paint_inheritance_zero_and_style_precedence_are_exact() {
    let cases = [
        (
            "<svg opacity=\"1e-200\"><g opacity=\"1e-200\"><path d=\"M0 0h1v1h-1z\"/></g></svg>",
            false,
        ),
        (
            "<svg opacity=\"1e-400%\"><g opacity=\"1e-400\"><path d=\"M0 0h1v1h-1z\"/></g></svg>",
            false,
        ),
        (
            "<svg opacity=\"0\"><g opacity=\"1\"><path d=\"M0 0h1v1h-1z\"/></g></svg>",
            true,
        ),
        (
            "<svg opacity=\"1e-400\"><g opacity=\"0\"><path d=\"M0 0h1v1h-1z\"/></g></svg>",
            true,
        ),
        (
            "<svg fill-opacity=\"0\"><path fill-opacity=\"1e-400\" d=\"M0 0h1v1h-1z\"/></svg>",
            false,
        ),
        (
            "<svg fill-opacity=\"1e-400\"><path fill-opacity=\"0\" d=\"M0 0h1v1h-1z\"/></svg>",
            true,
        ),
        (
            "<svg><path opacity=\"1e-400\" style=\"opacity:0\" d=\"M0 0h1v1h-1z\"/></svg>",
            true,
        ),
        ("<svg><path opacity=\"-0\" d=\"M0 0h1v1h-1z\"/></svg>", true),
        (
            "<svg><path opacity=\"-0%\" d=\"M0 0h1v1h-1z\"/></svg>",
            true,
        ),
        (
            "<svg><path fill=\"none\" stroke=\"black\" stroke-width=\"-0\" d=\"M0 0L1 1\"/></svg>",
            true,
        ),
        (
            "<svg stroke-opacity=\"0\" stroke-width=\"0\"><path fill=\"none\" stroke=\"black\" stroke-opacity=\"1e-400\" stroke-width=\"1e-400\" d=\"M0 0L1 1\"/></svg>",
            false,
        ),
        (
            "<svg stroke-opacity=\"1e-400\"><path fill=\"none\" stroke=\"black\" stroke-opacity=\"0\" d=\"M0 0L1 1\"/></svg>",
            true,
        ),
        (
            "<svg display=\"none\"><path opacity=\"1e-400\" d=\"M0 0h1v1h-1z\"/></svg>",
            true,
        ),
        (
            "<svg visibility=\"hidden\"><path visibility=\"visible\" opacity=\"1e-400\" d=\"M0 0h1v1h-1z\"/></svg>",
            false,
        ),
        (
            "<svg><path opacity=\"100%\" fill-opacity=\".5\" d=\"M0 0h1v1h-1z\"/></svg>",
            false,
        ),
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (document, empty) in cases {
            let outcome = support::under(&policy, || import_svg_document(document)).unwrap();
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            assert_eq!(outcome.value.is_empty(), empty, "{document}");
        }
    }
}

#[test]
fn svg_positive_paint_survives_native_replay_and_further_boolean() {
    const REPLAY: &str = "HYPERCURVE_POSITIVE_PAINT_REPLAY";
    let replay = std::env::var(REPLAY).ok();
    let source = "<svg opacity=\"1e-200\"><g opacity=\"1e-200\"><path fill-opacity=\"1e-400%\" d=\"M0 0h1v1h-1z\"/></g></svg>";
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome =
            support::under(&policy, || {
                let imported = import_svg_document(replay.as_deref().unwrap_or(source))?;
                assert!(!imported.is_empty());
                let expected = unit_square();
                assert!(
                    imported
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                let text = export_svg_document(&imported)?;
                #[cfg(not(target_arch = "wasm32"))]
                if replay.is_none() {
                    let child = std::process::Command::new(std::env::current_exe().unwrap())
                        .args([
                            "--exact",
                            "svg_positive_paint_survives_native_replay_and_further_boolean",
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
                        "fresh replay did not execute its witness: {stdout}"
                    );
                    for line in stdout.lines() {
                        println!("fresh paint replay: {line}");
                    }
                }
                let reloaded = import_svg_document(&text)?;
                assert!(
                    reloaded
                        .region()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                assert_eq!(export_svg_document(&reloaded)?, text);
                let reflection =
                    Similarity2::try_from_real_affine(r!(-1), r!(0), r!(0), r!(1), r!(3), r!(0))
                        .unwrap();
                let moved = reloaded.region().transform_similarity(&reflection).unwrap();
                let moved_expected = expected.transform_similarity(&reflection).unwrap();
                assert!(
                    moved
                        .boolean_region(&moved_expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
                let empty = moved
                    .boolean_region(&moved_expected, BooleanOp::Difference)
                    .unwrap();
                assert!(empty.is_empty());
                let restored = empty.boolean_region(&expected, BooleanOp::Union).unwrap();
                assert!(
                    restored
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
fn svg_extreme_scientific_paint_preserves_exact_sign() {
    let exponent = "9".repeat(400);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for name in ["opacity", "fill-opacity", "stroke-opacity", "stroke-width"] {
            for suffix in if name == "stroke-width" {
                &[""][..]
            } else {
                &["", "%"][..]
            } {
                let shape = if matches!(name, "stroke-opacity" | "stroke-width") {
                    "fill=\"none\" stroke=\"black\" d=\"M0 0L1 1\""
                } else {
                    "d=\"M0 0h1v1h-1z\""
                };
                for (significand, expected_empty) in [("1", false), ("0", true)] {
                    let document = format!(
                        "<svg><path {name}=\"{significand}e-{exponent}{suffix}\" {shape}/></svg>"
                    );
                    let outcome = support::under(&policy, || {
                        let imported = import_svg_document(&document)?;
                        assert_eq!(imported.is_empty(), expected_empty, "{name}{suffix}");
                        if !expected_empty {
                            if matches!(name, "stroke-opacity" | "stroke-width") {
                                assert!(imported.region().is_empty());
                                assert!(imported.paths().is_empty());
                                assert_eq!(imported.wires().len(), 1);
                                let [Segment2::Line(line)] = imported.wires()[0].segments() else {
                                    panic!("the exact positive factor must retain one line");
                                };
                                assert_eq!(line.start(), &pt!(0, 0));
                                assert_eq!(line.end(), &pt!(1, 1));
                            } else {
                                assert!(imported.wires().is_empty());
                                assert!(imported.paths().is_empty());
                                assert!(
                                    imported
                                        .region()
                                        .boolean_region(&unit_square(), BooleanOp::Xor)
                                        .unwrap()
                                        .is_empty()
                                );
                            }
                        }
                        Ok::<_, SvgError>(())
                    })
                    .unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                }
                let document =
                    format!("<svg><path {name}=\"-1e-{exponent}{suffix}\" {shape}/></svg>");
                let error = support::under(&policy, || import_svg_document(&document)).unwrap_err();
                assert!(
                    matches!(error, SvgError::MalformedInput(_)),
                    "{name}{suffix}: {error:?}"
                );
            }
        }
    }
}
