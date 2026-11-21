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

#[test]
fn rebound_join_models_certify_the_entire_axial_tangent_box() {
    use hyperlimit::PredicatePolicy;
    use hypersolve::{
        IntervalBoxCertificationPackage as Package, IntervalBoxCertificationStatus as Status,
        VariableBall, certify_interval_box_candidate, context_from_problem,
    };

    for target in [-3, 0, 2] {
        let join = g1_join_problem(axial(0), axial(0), axial(0), axial(target));
        let analysis = join.problem.analyze();
        let mut context = context_from_problem(&join.problem);
        for center in [-3, -1, 0, 1, 3] {
            context.bind(join.tangent_x_symbol, center.into());
            for radius in [0, 2] {
                // Endpoints and cross product agree everywhere. Orientation is
                // strict and its two extrema occur at the ends of this interval.
                let ends = [target * (center - radius), target * (center + radius)];
                let expected = if ends.iter().all(|&dot| dot > 0) {
                    Status::Certified
                } else if ends.iter().all(|&dot| dot <= 0) {
                    Status::Violation
                } else {
                    Status::Unknown
                };
                let bounds = [VariableBall {
                    symbol: join.tangent_x_symbol,
                    radius: radius.into(),
                }];
                for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                    for package in [
                        Package::Affine,
                        Package::UnivariateQuadratic,
                        Package::MultivariateQuadratic,
                    ] {
                        let report = certify_interval_box_candidate(
                            &analysis, &context, &bounds, package, policy,
                        );
                        assert_eq!(
                            report.status, expected,
                            "{target} {center} {radius} {package:?} {policy:?}"
                        );
                        assert_eq!(report.variable_balls, bounds);
                        let proof = report.certification.unwrap();
                        assert_eq!(proof.rows.len(), 4);
                        assert_eq!(proof.all_satisfied(), expected == Status::Certified);
                        assert_eq!(
                            proof.has_certified_violation(),
                            expected == Status::Violation
                        );
                    }
                }
            }
        }
    }
}
