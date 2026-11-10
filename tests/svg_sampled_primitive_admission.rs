#![cfg(feature = "svg-io")]

use csgrs::io::svg::{export_svg, import_svg};
use hypercurve::{BooleanOp, Curve2, CurvePath2, CurveRegion2, FillRule, Point2};

#[test]
fn svg_adapter_validates_empty_circle_and_ellipse_selection_before_continuation() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let points =
                [(0, 0), (1, 0), (1, 1), (0, 1)].map(|(x, y)| Point2::new(x.into(), y.into()));
            let expected = CurveRegion2::try_from_boundary_paths(
                &[CurvePath2::try_new(
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
                .unwrap()],
                FillRule::NonZero,
            )
            .unwrap();
            for document in [
                r#"<svg><circle fill="none" stroke="none" r="1e-400"/></svg>"#,
                r#"<svg><ellipse fill="none" stroke="none" rx="1" ry="1e-400"/></svg>"#,
                r#"<svg><circle fill="none" cx="1e308" cy="1e308" r="1e308"/></svg>"#,
                r#"<svg opacity="0"><ellipse rx="1e308" ry="1e308"/></svg>"#,
                r#"<svg><circle display="none" r="0"/></svg>"#,
            ] {
                let (empty, wires, paths) = import_svg(document)?;
                assert!(empty.is_empty() && wires.is_empty() && paths.is_empty());
                let text = export_svg(&empty, &wires, &paths)?;
                let (replayed, wires, paths) = import_svg(&text)?;
                assert!(replayed.is_empty() && wires.is_empty() && paths.is_empty());
                assert_eq!(export_svg(&replayed, &wires, &paths)?, text);
                assert!(
                    replayed
                        .boolean_region(&expected, BooleanOp::Union)
                        .unwrap()
                        .boolean_region(&expected, BooleanOp::Xor)
                        .unwrap()
                        .is_empty()
                );
            }
            for document in [
                r#"<svg opacity="0"><circle r="-1e-400"/></svg>"#,
                r#"<svg><ellipse display="none" rx="1" ry="-1e-400"/></svg>"#,
                r#"<svg><circle visibility="hidden" cx="inf" r="1"/></svg>"#,
            ] {
                assert!(
                    matches!(import_svg(document),
                    Err(csgrs::io::IoError::MalformedInput(detail)) if !detail.is_empty()),
                    "{document}"
                );
            }
            Ok::<_, csgrs::io::IoError>(())
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.into_unverified().unwrap();
    }
}

#[test]
fn svg_adapter_keeps_positive_circle_and_ellipse_underflow_unsupported() {
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            for shape in [
                r#"<circle r="1e-400"/>"#,
                r#"<ellipse rx="1e-400" ry="1"/>"#,
                r#"<ellipse rx="1" ry="1e-400"/>"#,
            ] {
                let document = format!("<svg>{shape}</svg>");
                let error = match import_svg(&document) {
                    Err(error) => error,
                    Ok(_) => panic!("sampler limit must be explicit for {document}"),
                };
                assert!(
                    matches!(error, csgrs::io::IoError::Unsupported { format: "SVG", .. }),
                    "{document}: {error:?}"
                );
            }
            Ok::<_, csgrs::io::IoError>(())
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.into_unverified().unwrap();
    }
}
