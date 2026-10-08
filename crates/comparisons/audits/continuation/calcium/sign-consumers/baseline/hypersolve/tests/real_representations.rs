//! Exhaustive Hyperreal representation coverage at Hypersolve's public boundary.
//!
//! Hyperreal exposes eight public structural kinds, while its optimized
//! finite `Real` representation currently has 22 certificate classes. This
//! suite keeps a fail-closed constructor inventory and sends every class
//! through symbolic analysis, exact evaluation, candidate/domain proof,
//! Jacobian construction, direct solving, and dense/sparse exact replay.

use std::collections::HashMap;

use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::{
    CertifiedRealEquality, CertifiedRealSign, Rational, Real, RealSign, StructuralKind,
};
use hypersolve::jacobian::{symbolic_jacobian, symbolic_jacobian_with_analysis};
use hypersolve::predicates::StructuralPredicateBackend;
use hypersolve::{
    BareissError, Classification, Constraint, ConstraintKind, DomainCheckStatus, EvaluationContext,
    Expr, ExprDegree, PredicateBackend, Problem, SparseBareissError, SparseResidualReplayError,
    SparseResidualTerm, SymbolId, certify_candidate, certify_candidate_domains,
    classify_real_sign_predicate, compare_real_predicate, context_from_problem,
    determinant_bareiss, evaluate_residuals, replay_dense_linear_residuals,
    replay_sparse_linear_residuals, solve_dense_linear_system_bareiss, solve_direct_affine_system,
    solve_sparse_linear_system_bareiss, solve_sparse_linear_system_bareiss_pattern_preserving,
};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;
const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const MIN_PRECISION: i32 = -512;

#[derive(Clone)]
struct RepresentationCase {
    certificate: &'static str,
    public_kind: StructuralKind,
    value: Real,
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn representation_cases() -> Vec<RepresentationCase> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        RepresentationCase {
            certificate: "One",
            public_kind: StructuralKind::ExactRational,
            value: fraction(3, 2),
        },
        RepresentationCase {
            certificate: "Pi",
            public_kind: StructuralKind::PiLike,
            value: pi.clone(),
        },
        RepresentationCase {
            certificate: "PiPow",
            public_kind: StructuralKind::PiLike,
            value: pi_squared.clone(),
        },
        RepresentationCase {
            certificate: "PiInv",
            public_kind: StructuralKind::PiLike,
            value: pi.clone().inverse().expect("pi is nonzero"),
        },
        RepresentationCase {
            certificate: "PiExp",
            public_kind: StructuralKind::ExpLike,
            value: &pi * &e,
        },
        RepresentationCase {
            certificate: "PiInvExp",
            public_kind: StructuralKind::ExpLike,
            value: (&e / &pi).expect("pi is nonzero"),
        },
        RepresentationCase {
            certificate: "PiSqrt",
            public_kind: StructuralKind::SqrtLike,
            value: &pi * &sqrt_two,
        },
        RepresentationCase {
            certificate: "ConstProduct",
            public_kind: StructuralKind::ProductConstant,
            value: &pi_squared * &e,
        },
        RepresentationCase {
            certificate: "ConstOffset",
            public_kind: StructuralKind::ProductConstant,
            value: &pi - Real::from(3),
        },
        RepresentationCase {
            certificate: "ConstProductSqrt",
            public_kind: StructuralKind::ProductConstant,
            value: &(&pi_squared * &e) * &sqrt_two,
        },
        RepresentationCase {
            certificate: "Sqrt",
            public_kind: StructuralKind::SqrtLike,
            value: sqrt_two,
        },
        RepresentationCase {
            certificate: "Exp",
            public_kind: StructuralKind::ExpLike,
            value: Real::from(2).exp().expect("finite exponential"),
        },
        RepresentationCase {
            certificate: "Ln",
            public_kind: StructuralKind::LogLike,
            value: ln_three.clone(),
        },
        RepresentationCase {
            certificate: "LnAffine",
            public_kind: StructuralKind::LogLike,
            value: (Real::from(2) * &e).ln().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "LnProduct",
            public_kind: StructuralKind::LogLike,
            value: &ln_two * &ln_three,
        },
        RepresentationCase {
            certificate: "Log10",
            public_kind: StructuralKind::LogLike,
            value: Real::from(2).log10().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "Log2",
            public_kind: StructuralKind::LogLike,
            value: Real::from(3).log2().expect("positive logarithm input"),
        },
        RepresentationCase {
            certificate: "Pow10",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        },
        RepresentationCase {
            certificate: "Pow2",
            public_kind: StructuralKind::ExpLike,
            value: fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        },
        RepresentationCase {
            certificate: "SinPi",
            public_kind: StructuralKind::TrigExact,
            value: fraction(1, 5).sin_pi(),
        },
        RepresentationCase {
            certificate: "TanPi",
            public_kind: StructuralKind::TrigExact,
            value: fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        },
        RepresentationCase {
            certificate: "Irrational",
            public_kind: StructuralKind::ComputableOpaque,
            value: Real::one().sin(),
        },
    ]
}

fn structural_kind_index(kind: StructuralKind) -> usize {
    match kind {
        StructuralKind::ExactRational => 0,
        StructuralKind::PiLike => 1,
        StructuralKind::ExpLike => 2,
        StructuralKind::SqrtLike => 3,
        StructuralKind::LogLike => 4,
        StructuralKind::TrigExact => 5,
        StructuralKind::ProductConstant => 6,
        StructuralKind::ComputableOpaque => 7,
    }
}

fn assert_zero(value: &Real, context: &str) {
    assert!(
        matches!(
            value.certified_sign_until(MIN_PRECISION),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            }
        ),
        "{context}: expected certified zero, got {value:?}"
    );
}

fn assert_same_real(left: &Real, right: &Real, context: &str) {
    if matches!(
        left.certified_eq_until(right, MIN_PRECISION),
        CertifiedRealEquality::Equal { .. }
    ) || matches!(
        (left - right).certified_sign_until(MIN_PRECISION),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    ) {
        return;
    }

    let [left_lower, left_upper] = left
        .certified_dyadic_interval(MIN_PRECISION)
        .unwrap_or_else(|| panic!("{context}: left value is not bounded"));
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(MIN_PRECISION)
        .unwrap_or_else(|| panic!("{context}: right value is not bounded"));
    assert!(
        left_lower <= right_upper && right_lower <= left_upper,
        "{context}: certified intervals for equal expressions do not overlap"
    );
}

fn affine_problem(coefficient: &Real) -> (Problem, EvaluationContext) {
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", Real::from(2));
    let symbol = SymbolId(variable.0);
    problem.add_constraint(Constraint::equality(
        "representation affine row",
        Expr::real(coefficient.clone()) * Expr::symbol(symbol, "x")
            - Expr::real(coefficient * Real::from(2)),
    ));
    let context = context_from_problem(&problem);
    (problem, context)
}

#[test]
fn finite_real_certificate_inventory_is_current() {
    let cases = representation_cases();
    assert_eq!(
        cases.len(),
        22,
        "update the private Real certificate matrix"
    );

    let mut observed_kinds = [false; 8];
    for case in cases {
        let facts = case.value.detailed_facts();
        assert_eq!(
            facts.symbolic.kind, case.public_kind,
            "{} recipe drifted",
            case.certificate
        );
        let debug = format!("{:?}", case.value);
        assert!(
            debug.contains(&format!("class: {}", case.certificate)),
            "{} recipe no longer constructs its named private certificate: {debug}",
            case.certificate
        );
        observed_kinds[structural_kind_index(facts.symbolic.kind)] = true;
    }
    assert_eq!(observed_kinds, [true; 8], "missing public Real kind");
}

#[test]
fn every_real_certificate_crosses_solver_analysis_and_proof_surfaces() {
    let structural_backend = StructuralPredicateBackend;
    let mut sparse_solved = 0;
    let mut sparse_explicitly_unknown = 0;
    let mut pattern_solved = 0;
    let mut pattern_explicitly_unknown = 0;

    for case in representation_cases() {
        let context_label = case.certificate;
        let value = &case.value;
        let mut problem = Problem::default();
        let variable = problem.add_variable("x", value.clone());
        let symbol = SymbolId(variable.0);
        let x = || Expr::symbol(symbol, "x");
        let delta = || x() - Expr::real(value.clone());

        problem.add_constraint(Constraint::equality("affine", delta()));
        problem.add_constraint(Constraint::equality("quadratic", delta() * delta()));
        problem.add_constraint(Constraint::equality("non-polynomial", delta().sin()));
        problem.add_constraint(Constraint {
            name: "less-or-equal".into(),
            kind: ConstraintKind::LessOrEqual,
            residual: delta(),
            weight: value.clone(),
            active: true,
        });
        problem.add_constraint(Constraint {
            name: "greater-or-equal".into(),
            kind: ConstraintKind::GreaterOrEqual,
            residual: delta(),
            weight: value.clone(),
            active: true,
        });

        let analysis = problem.analyze();
        assert_eq!(
            analysis.facts().active_constraint_count,
            5,
            "{context_label}"
        );
        assert_eq!(
            analysis.constraints()[0].residual.degree,
            ExprDegree::Polynomial(1),
            "{context_label}"
        );
        assert_eq!(
            analysis.constraints()[1].residual.degree,
            ExprDegree::Polynomial(2),
            "{context_label}"
        );
        assert_eq!(
            analysis.constraints()[2].residual.degree,
            ExprDegree::NonPolynomial,
            "{context_label}"
        );
        assert_eq!(analysis.affine_residuals().iter().flatten().count(), 3);
        assert_eq!(analysis.quadratic_residuals().iter().flatten().count(), 1);
        assert!(
            analysis
                .jacobian_sparsity()
                .iter()
                .all(|row| row == &[true])
        );

        let context = context_from_problem(&problem);
        for residual in evaluate_residuals(&problem, &context).expect("tree evaluation") {
            assert_zero(&residual.value, context_label);
        }
        for residual in analysis
            .evaluate_residuals(&context)
            .expect("retained-package evaluation")
        {
            assert_zero(&residual.value, context_label);
        }
        let certification = certify_candidate(&analysis, &context);
        assert!(certification.all_satisfied(), "{context_label}");
        assert_eq!(certification.certified_satisfied_rows, 5);

        let ordinary_jacobian = symbolic_jacobian(&problem, &context).expect("symbolic Jacobian");
        let retained_jacobian =
            symbolic_jacobian_with_analysis(&analysis, &context).expect("retained Jacobian");
        assert_eq!(ordinary_jacobian, retained_jacobian, "{context_label}");
        assert_eq!(retained_jacobian.len(), 5);
        assert!(retained_jacobian.iter().all(|row| row.len() == 1));

        let structural = structural_backend.classify_sign(value);
        assert!(
            matches!(
                structural.classification,
                Classification::Satisfied | Classification::Unknown
            ),
            "{context_label}"
        );
        assert!(matches!(
            classify_real_sign_predicate(value, STRICT),
            PredicateOutcome::Decided {
                value: Sign::Positive,
                ..
            }
        ));
        assert!(matches!(
            compare_real_predicate(value, &Real::zero(), APPROX),
            PredicateOutcome::Decided {
                value: std::cmp::Ordering::Greater,
                ..
            }
        ));

        let (affine, affine_context) = affine_problem(value);
        let affine_analysis = affine.analyze();
        let direct = solve_direct_affine_system(&affine_analysis);
        assert!(direct.solved(), "{context_label}: {:?}", direct.status);
        assert_eq!(direct.assignments.len(), 1);
        assert_same_real(&direct.assignments[0].value, &Real::from(2), context_label);
        assert!(
            certify_candidate(&affine_analysis, &affine_context).all_satisfied(),
            "{context_label}"
        );

        let matrix = vec![
            vec![value.clone(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        let rhs = vec![value * Real::from(2), Real::from(3)];
        let determinant = determinant_bareiss(&matrix, MIN_PRECISION).expect("determinant");
        assert_same_real(&determinant.determinant, value, context_label);

        let dense = solve_dense_linear_system_bareiss(&matrix, &rhs, MIN_PRECISION, APPROX)
            .unwrap_or_else(|error| panic!("{context_label}: dense Bareiss failed: {error:?}"));
        assert!(dense.residual_replay.accepted, "{context_label}");
        assert_same_real(&dense.solution[0], &Real::from(2), context_label);
        assert_same_real(&dense.solution[1], &Real::from(3), context_label);

        // Keep the strict sparse proof identity-preserving. Unlike the dense
        // API above, this API intentionally has no approximate terminal
        // policy and must not accept merely overlapping computable DAGs.
        let terms = vec![SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: value.clone(),
        }];
        let sparse_rhs = vec![value.clone()];
        match solve_sparse_linear_system_bareiss(1, 1, &terms, &sparse_rhs, MIN_PRECISION) {
            Ok(sparse) => {
                assert!(
                    sparse.dense_solve.residual_replay.accepted,
                    "{context_label}"
                );
                assert!(sparse.sparse_residual_replay.accepted, "{context_label}");
                assert_same_real(&sparse.dense_solve.solution[0], &Real::one(), context_label);
                sparse_solved += 1;
            }
            Err(SparseBareissError::DenseSolve(BareissError::UnknownResidual))
            | Err(SparseBareissError::SparseReplay(SparseResidualReplayError::UnknownResidual)) => {
                // The strict API must surface non-canonical computable
                // cancellation as uncertainty, never silently use f64.
                sparse_explicitly_unknown += 1;
            }
            Err(error) => panic!("{context_label}: unexpected sparse solve failure: {error:?}"),
        }

        match solve_sparse_linear_system_bareiss_pattern_preserving(
            1,
            1,
            &terms,
            &sparse_rhs,
            MIN_PRECISION,
        ) {
            Ok(pattern) => {
                assert!(pattern.sparse_residual_replay.accepted, "{context_label}");
                assert_same_real(&pattern.solution[0], &Real::one(), context_label);
                pattern_solved += 1;
            }
            Err(SparseBareissError::SparseReplay(SparseResidualReplayError::UnknownResidual)) => {
                pattern_explicitly_unknown += 1
            }
            Err(error) => {
                panic!("{context_label}: unexpected pattern-preserving failure: {error:?}")
            }
        }

        let mut domain_problem = Problem::default();
        let domain_variable = domain_problem.add_variable("positive", value.clone());
        let domain_symbol = SymbolId(domain_variable.0);
        domain_problem.add_constraint(Constraint::equality(
            "positive domains",
            Expr::symbol(domain_symbol, "positive").ln()
                + Expr::symbol(domain_symbol, "positive").sqrt()
                + (Expr::int(1) / Expr::symbol(domain_symbol, "positive")),
        ));
        let domain_context = context_from_problem(&domain_problem);
        let domain = certify_candidate_domains(&domain_problem, &domain_context, APPROX);
        assert_eq!(domain.checks.len(), 3, "{context_label}");
        assert!(
            domain.all_checks_certified_valid(),
            "{context_label}: {domain:?}"
        );
        assert!(
            domain
                .checks
                .iter()
                .all(|check| check.status == DomainCheckStatus::CertifiedValid)
        );
    }

    assert_eq!(sparse_solved + sparse_explicitly_unknown, 22);
    assert_eq!(pattern_solved + pattern_explicitly_unknown, 22);
    assert!(sparse_solved > 0 && pattern_solved > 0);
    assert!(sparse_explicitly_unknown > 0);
}

#[test]
fn every_ordered_pair_of_real_certificates_crosses_expression_and_replay() {
    let cases = representation_cases();
    let x = SymbolId(0);
    let y = SymbolId(1);
    let mut exercised = 0;

    for left in &cases {
        for right in &cases {
            let context_label = format!("{} with {}", left.certificate, right.certificate);
            let mut bindings = HashMap::new();
            bindings.insert(x, left.value.clone());
            bindings.insert(y, right.value.clone());

            let sum = Expr::symbol(x, "x") + Expr::symbol(y, "y")
                - Expr::real(&left.value + &right.value);
            let difference = Expr::symbol(x, "x")
                - Expr::symbol(y, "y")
                - Expr::real(&left.value - &right.value);
            let product = Expr::symbol(x, "x") * Expr::symbol(y, "y")
                - Expr::real(&left.value * &right.value);
            let quotient = Expr::symbol(x, "x") / Expr::symbol(y, "y")
                - Expr::real((&left.value / &right.value).expect("representatives are nonzero"));

            for expression in [&sum, &difference, &product, &quotient] {
                assert_eq!(expression.structural_facts().dependencies.len(), 2);
                let actual = expression
                    .eval_real(&bindings)
                    .unwrap_or_else(|error| panic!("{context_label}: {error:?}"));
                // Equivalent computable DAGs are not required to canonicalize
                // to Hyperreal's structural zero class. Their certified
                // enclosures must still agree with mathematical zero.
                assert_same_real(&actual, &Real::zero(), &context_label);
            }

            let matrix = [
                vec![left.value.clone(), Real::zero()],
                vec![Real::zero(), right.value.clone()],
            ];
            let rhs = [&left.value * Real::from(2), &right.value * Real::from(3)];
            let replay = replay_dense_linear_residuals(
                &matrix,
                &rhs,
                &[Real::from(2), Real::from(3)],
                MIN_PRECISION,
                APPROX,
            )
            .unwrap_or_else(|error| panic!("{context_label}: dense replay failed: {error:?}"));
            assert!(replay.accepted, "{context_label}");

            let sparse_terms = [
                SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: left.value.clone(),
                },
                SparseResidualTerm {
                    row: 1,
                    column: 1,
                    coefficient: right.value.clone(),
                },
            ];
            let sparse = replay_sparse_linear_residuals(
                2,
                2,
                &sparse_terms,
                &rhs,
                &[Real::from(2), Real::from(3)],
                MIN_PRECISION,
            )
            .unwrap_or_else(|error| panic!("{context_label}: sparse replay failed: {error:?}"));
            assert!(sparse.accepted, "{context_label}");
            exercised += 1;
        }
    }

    assert_eq!(exercised, 22 * 22, "ordered-pair matrix is incomplete");
}

fn opaque_graph_values(depth_seed: u8, opcode_seed: u8) -> Vec<Real> {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let identity_residual = &sine * &sine + &cosine * &cosine - Real::one();
    let mut recursive = &sine + &cosine;

    for level in 0..=usize::from(depth_seed % 8) {
        recursive = match (usize::from(opcode_seed) + level) % 6 {
            0 => recursive.sin(),
            1 => recursive.cos(),
            2 => recursive.exp().expect("bounded finite graph exponential"),
            3 => &recursive * &recursive + &sine,
            4 => &recursive + &cosine,
            5 => (&recursive * &sine) - &cosine,
            _ => unreachable!(),
        };
    }

    let values = vec![sine, cosine, identity_residual, recursive];
    assert!(
        values.iter().all(|value| {
            value.detailed_facts().symbolic.kind == StructuralKind::ComputableOpaque
        })
    );
    values
}

#[test]
fn variable_depth_opaque_dags_cross_solver_evaluation_and_certification() {
    for depth in 0..=7 {
        for (index, value) in opaque_graph_values(depth, depth.wrapping_mul(17))
            .into_iter()
            .enumerate()
        {
            let context_label = format!("opaque depth {depth} value {index}");
            assert!(value.certified_dyadic_interval(MIN_PRECISION).is_some());
            let mut problem = Problem::default();
            let variable = problem.add_variable("opaque", value.clone());
            let symbol = SymbolId(variable.0);
            problem.add_constraint(Constraint::equality(
                "opaque identity",
                Expr::symbol(symbol, "opaque") - Expr::real(value),
            ));
            let analysis = problem.analyze();
            let context = context_from_problem(&problem);
            let report = certify_candidate(&analysis, &context);
            assert!(report.all_satisfied(), "{context_label}: {report:?}");
        }
    }
}
