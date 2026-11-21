use hyperlimit::Point2;
use hyperpath::{
    TangentSpan, certify_g1_chain, certify_g1_join_candidate, certify_tangent_alignment_candidate,
    g1_join_problem, oriented_tangent_alignment_problem, tangent_alignment_problem,
};

fn axial(x: i32) -> Point2 {
    Point2::new(x.into(), 0.into())
}

#[test]
fn oriented_models_require_a_strictly_positive_dot_product() {
    for x in [-2, 0, 3] {
        for y in [-3, 0, 5] {
            let inside = x * y > 0;
            let span = |t| TangentSpan {
                start: axial(0),
                end: axial(0),
                start_tangent: axial(t),
                end_tangent: axial(t),
            };
            let oriented = oriented_tangent_alignment_problem(axial(x), axial(y));
            let join = g1_join_problem(axial(0), axial(x), axial(0), axial(y));
            let chain = certify_g1_chain(&[span(x), span(y)]);
            assert_eq!(chain.all_certified(), inside);
            for report in [
                certify_g1_join_candidate(&join),
                certify_tangent_alignment_candidate(&oriented),
                chain.joins.into_iter().next().unwrap(),
            ] {
                assert_eq!(report.all_satisfied(), inside);
                assert_eq!(report.has_certified_violation(), !inside);
            }
            // The unoriented model asks only for the cross-product residual.
            let parallel = tangent_alignment_problem(axial(x), axial(y));
            assert!(certify_tangent_alignment_candidate(&parallel).all_satisfied());
        }
    }
}
