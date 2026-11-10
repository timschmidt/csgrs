#![cfg(feature = "svg")]

mod support;

use hypercurve::{
    BooleanOp, Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, Point2, SvgError,
    export_svg_document, import_svg_document,
};

#[test]
fn svg_hidden_zero_radius_primitives_preserve_existing_empty_selection() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        for selection in [
            r#"opacity="0""#,
            r#"display="none""#,
            r#"visibility="hidden""#,
        ] {
            for shape in [
                r#"<circle r="0""#,
                r#"<ellipse rx="0" ry="1""#,
                r#"<ellipse rx="1" ry="0""#,
                r#"<ellipse rx="0" ry="0""#,
            ] {
                let document = format!("<svg>{shape} {selection}/></svg>");
                let outcome = support::under(&policy, || import_svg_document(&document)).unwrap();
                assert!(outcome.value.is_empty(), "{document}");
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
            }
        }
    }
}

#[test]
fn svg_hidden_circles_and_ellipses_validate_their_authored_fields() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        for selection in [
            "",
            r#"fill="none" stroke="none""#,
            r#"opacity="0""#,
            r#"display="none""#,
            r#"visibility="hidden""#,
        ] {
            for (tag, attributes) in [
                ("circle", r#"r="-1""#),
                ("circle", r#"r="-1e-400""#),
                ("circle", r#"r="NaN""#),
                ("circle", r#"cx="inf" r="1""#),
                ("circle", r#"cy="NaN" r="1""#),
                ("circle", r#"r="1em""#),
                ("circle", r#"r="1e400""#),
                ("circle", ""),
                ("ellipse", r#"rx="-1e-400" ry="1""#),
                ("ellipse", r#"rx="1" ry="-1e-400""#),
                ("ellipse", r#"rx="NaN" ry="1""#),
                ("ellipse", r#"rx="1" ry="inf""#),
                ("ellipse", r#"cx="NaN" rx="1" ry="1""#),
                ("ellipse", r#"cy="inf" rx="1" ry="1""#),
                ("ellipse", r#"rx="1/2" ry="1""#),
                ("ellipse", r#"rx="1e400" ry="1""#),
                ("ellipse", r#"rx="1""#),
            ] {
                let document = format!("<svg><{tag} {selection} {attributes}/></svg>");
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
fn svg_unpainted_positive_circles_and_ellipses_do_not_require_sampling() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        for selection in [
            r#"fill="none" stroke="none""#,
            r#"opacity="0""#,
            r#"display="none""#,
            r#"visibility="hidden""#,
        ] {
            for (tag, attributes) in [
                ("circle", r#"r="1e-400""#),
                ("circle", r#"cx="1e308" cy="1e308" r="1e308""#),
                ("ellipse", r#"rx="1e-400" ry="1""#),
                ("ellipse", r#"rx="1" ry="1e-400""#),
                ("ellipse", r#"cx="1e308" cy="1e308" rx="1e308" ry="1e308""#),
            ] {
                let document = format!("<svg><{tag} {selection} {attributes}/></svg>");
                let outcome = support::under(&policy, || {
                    let empty = import_svg_document(&document)?;
                    assert!(empty.is_empty(), "{document}");
                    assert!(empty.region().is_empty());
                    assert!(empty.paths().is_empty() && empty.wires().is_empty());
                    let replay = import_svg_document(&export_svg_document(&empty)?)?;
                    assert!(replay.is_empty());
                    // Independent native unit-square supports; no SVG parser
                    // supplies the expected geometry of the subsequent union.
                    let points = [(0, 0), (1, 0), (1, 1), (0, 1)]
                        .map(|(x, y)| Point2::new(x.into(), y.into()));
                    let path = CurvePath2::try_new(
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
                    let square =
                        CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
                    let recovered = replay
                        .region()
                        .boolean_region(&square, BooleanOp::Union)
                        .unwrap();
                    assert!(
                        recovered
                            .boolean_region(&square, BooleanOp::Xor)
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

#[test]
fn svg_visible_positive_circle_sampler_underflow_is_unsupported() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        for shape in [
            r#"<circle r="1e-400"/>"#,
            r#"<ellipse rx="1e-400" ry="1"/>"#,
            r#"<ellipse rx="1" ry="1e-400"/>"#,
        ] {
            let document = format!("<svg>{shape}</svg>");
            assert!(
                matches!(
                    support::under(&policy, || import_svg_document(&document)),
                    Err(SvgError::Unsupported(detail)) if !detail.is_empty()
                ),
                "{document}"
            );
        }
    }
}

#[test]
fn svg_scientific_radius_selection_does_not_materialize_large_scales() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        for shape in [
            r#"<circle r="1e-1000000000"/>"#,
            r#"<ellipse rx="1e-1000000000" ry="1"/>"#,
        ] {
            let hidden = format!(r#"<svg fill="none" stroke="none">{shape}</svg>"#);
            let outcome = support::under(&policy, || import_svg_document(&hidden)).unwrap();
            assert!(outcome.value.is_empty());
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            let visible = format!("<svg>{shape}</svg>");
            assert!(matches!(
                support::under(&policy, || import_svg_document(&visible)),
                Err(SvgError::Unsupported(detail)) if !detail.is_empty()
            ));
        }
        for shape in [
            r#"<circle display="none" r="-1e-1000000000"/>"#,
            r#"<ellipse visibility="hidden" rx="1" ry="-1e-1000000000"/>"#,
        ] {
            let document = format!("<svg>{shape}</svg>");
            assert!(matches!(
                support::under(&policy, || import_svg_document(&document)),
                Err(SvgError::MalformedInput(detail)) if !detail.is_empty()
            ));
        }
    }
}
