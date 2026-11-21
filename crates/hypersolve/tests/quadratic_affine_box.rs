//! Whole-box scope of affine rows alongside retained quadratic residuals.
use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::{Real, RealSign};
use hypersolve::{
    CertifiedCandidateStatus, Constraint, ConstraintKind, Expr,
    IntervalBoxCertificationPackage as Package, IntervalBoxCertificationStatus as Status, Problem,
    SymbolId, VariableBall, certify_interval_box_candidate, context_from_problem,
};

#[test]
fn mixed_degree_boxes_enclose_each_active_affine_row_in_source_coordinates() {
    let mut p = Problem::default();
    let x = SymbolId::from(p.add_variable("x", Real::zero()));
    let y = SymbolId::from(p.add_variable("y", Real::zero()));
    let var = Expr::anonymous_symbol;
    let mut inactive = Constraint::equality("ignored", Expr::int(1));
    inactive.active = false;
    p.add_constraint(inactive);
    let mut quad = Constraint::equality("bounded quadratic", var(x) * var(x) - Expr::int(1000));
    quad.kind = ConstraintKind::LessOrEqual;
    p.add_constraint(quad);
    let mut linear =
        Constraint::equality("half-space", Expr::int(3) * var(x) - Expr::int(2) * var(y));
    linear.kind = ConstraintKind::GreaterOrEqual;
    p.add_constraint(linear);

    let analysis = p.analyze();
    for (cx, cy, rx, ry) in [
        (3, 1, 1, 1),
        (-3, 1, 1, 1),
        (1, 1, 1, 1),
        (-1, -1, 1, 1),
        (0, 0, 1, 1),
    ] {
        let mut ctx = context_from_problem(&p);
        ctx.bind(x, Real::from(cx));
        ctx.bind(y, Real::from(cy));
        let balls = [
            VariableBall {
                symbol: y,
                radius: Real::from(ry),
            },
            VariableBall {
                symbol: x,
                radius: Real::from(rx),
            },
        ];
        // The four independently evaluated vertices attain both affine extrema.
        let corners = [cx - rx, cx + rx].map(|x| [cy - ry, cy + ry].map(|y| 3 * x - 2 * y));
        let expected = if corners.iter().flatten().all(|v| *v > 0) {
            Status::Certified
        } else if corners.iter().flatten().all(|v| *v < 0) {
            Status::Violation
        } else {
            Status::Unknown
        };
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for package in [Package::UnivariateQuadratic, Package::MultivariateQuadratic] {
                let report =
                    certify_interval_box_candidate(&analysis, &ctx, &balls, package, policy);
                assert_eq!(report.status, expected, "{package:?} {policy:?} {cx} {cy}");
                assert_eq!(report.variable_balls, balls);
                let proof = report.certification.unwrap();
                assert_eq!(proof.rows.len(), 2);
                assert_eq!(
                    (
                        proof.rows[0].constraint_index,
                        proof.rows[1].constraint_index
                    ),
                    (1, 2)
                );
                assert!(proof.rows[0].status.is_certified_satisfied());
                assert_eq!(proof.all_satisfied(), expected == Status::Certified);
                assert_eq!(
                    proof.has_certified_violation(),
                    expected == Status::Violation
                );
                let row = &proof.rows[1];
                assert_eq!(row.signed_residual, Some(Real::from(2 * cy - 3 * cx)));
                match (&row.status, &expected) {
                    (CertifiedCandidateStatus::BoundedUnknown { .. }, Status::Unknown) => {}
                    (
                        CertifiedCandidateStatus::BallCertified {
                            sign,
                            certainty,
                            satisfied,
                        },
                        _,
                    ) => {
                        assert!(matches!(certainty, Certainty::Exact | Certainty::Filtered));
                        assert_ne!(expected, Status::Unknown);
                        assert_eq!(*satisfied, expected == Status::Certified);
                        assert_eq!(
                            *sign,
                            if *satisfied {
                                RealSign::Negative
                            } else {
                                RealSign::Positive
                            }
                        );
                    }
                    other => panic!("missing whole-box evidence: {other:?}"),
                }
            }
        }
    }
}

include!("support/hidden_positive.rs");

#[test]
fn a_hidden_face_crossing_cannot_certify_the_half_space() {
    let mut p = Problem::default();
    let x = SymbolId::from(p.add_variable("x", Real::one()));
    let mut row = Constraint::equality(
        "lower bound",
        Expr::anonymous_symbol(x) - Expr::real(hidden_positive()),
    );
    row.kind = ConstraintKind::GreaterOrEqual;
    p.add_constraint(row);
    let analysis = p.analyze();
    let ctx = context_from_problem(&p);
    // At x=0 the row is negative and at x=2 it is positive. Its center alone satisfies it.
    let bounds = [VariableBall {
        symbol: x,
        radius: Real::one(),
    }];
    for policy in [PredicatePolicy::APPROXIMATE_512, PredicatePolicy::STRICT] {
        for package in [Package::UnivariateQuadratic, Package::MultivariateQuadratic] {
            let report = certify_interval_box_candidate(&analysis, &ctx, &bounds, package, policy);
            assert_eq!(report.status, Status::Unknown);
            let rows = report.certification.unwrap();
            assert!(rows.has_only_uncertainty());
            assert_eq!(rows.bounded_unknown_rows, 1);
        }
    }
}
