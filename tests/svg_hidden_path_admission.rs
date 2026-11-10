#![cfg(feature = "svg-io")]

use csgrs::io::{
    IoError,
    svg::{export_svg, import_svg},
};
use hypercurve::{BooleanOp, CurveCertainty, PredicatePolicy, evaluate_under};

#[test]
fn svg_adapter_keeps_hidden_path_errors_distinct_from_replayable_empty() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome = evaluate_under(policy, || {
            for document in [
                r#"<svg><path visibility="hidden"/></svg>"#,
                r#"<svg opacity="0"><path d="M0 0 L1"/></svg>"#,
                r#"<svg><path display="none" data-hypercurve-path="3:0" d="M0 0 H1 V1 H0 Z"/></svg>"#,
            ] {
                assert!(
                    matches!(import_svg(document), Err(IoError::MalformedInput(_))),
                    "{document}"
                );
            }
            let (empty, wires, paths) =
                import_svg(r#"<svg><path display="none" d="M0 0 A2 1 0 0 1 4 0 Z"/></svg>"#)?;
            assert!(empty.is_empty() && wires.is_empty() && paths.is_empty());
            let text = export_svg(&empty, &wires, &paths)?;
            let (replayed, wires, paths) = import_svg(&text)?;
            assert!(replayed.is_empty() && wires.is_empty() && paths.is_empty());
            assert_eq!(export_svg(&replayed, &wires, &paths)?, text);
            let (square, _, _) = import_svg(r#"<svg><rect width="1" height="1"/></svg>"#)?;
            assert!(!square.is_empty());
            assert!(
                replayed
                    .boolean_region(&square, BooleanOp::Union)
                    .unwrap()
                    .boolean_region(&square, BooleanOp::Xor)
                    .unwrap()
                    .is_empty()
            );
            Ok::<_, IoError>(())
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
        outcome.into_unverified().unwrap();
    }
}
