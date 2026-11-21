use hyperreal::Real;
use hypersolve::{
    Constraint, ConvergenceReason, DenseLinearBackend, Expr, LinearBackend, LinearSolveError,
    Problem, ProposalEngineKind, SolverConfig, SolverState, SymbolId, solve_damped_least_squares,
};

#[test]
fn all_adapters_reject_nonfinite_inputs_before_success() {
    let b = DenseLinearBackend;
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for (matrix, vector, parameter) in [
            (vec![vec![bad]], vec![1.0], 1.0),
            (vec![vec![1.0]], vec![bad], 1.0),
            (vec![vec![1.0]], vec![1.0], bad),
            (vec![], vec![], bad),
        ] {
            for result in [
                b.solve_damped_normal(&matrix, &vector, parameter),
                b.solve_dogleg(&matrix, &vector, parameter),
                b.solve_bfgs_direction(&matrix, &vector, parameter),
            ] {
                assert_eq!(result.unwrap_err(), LinearSolveError::NonFiniteInput);
            }
        }
        // Invalid dimensions retain their existing diagnostic priority.
        for result in [
            b.solve_damped_normal(&[vec![bad]], &[], bad),
            b.solve_dogleg(&[vec![bad]], &[], bad),
            b.solve_bfgs_direction(&[vec![bad]], &[], bad),
            b.solve_damped_normal(&[vec![], vec![bad]], &[0.0, 0.0], bad),
            b.solve_dogleg(&[vec![], vec![bad]], &[0.0, 0.0], bad),
        ] {
            assert_eq!(result.unwrap_err(), LinearSolveError::DimensionMismatch);
        }
    }
}

#[test]
fn finite_overflow_is_distinct_from_invalid_input_and_singularity() {
    let b = DenseLinearBackend;
    for result in [
        b.solve_damped_normal(&[vec![1e308]], &[1e308], 0.0),
        b.solve_dogleg(&[vec![1e308]], &[1e308], 1.0),
        b.solve_bfgs_direction(&[vec![1e308]], &[2.0], 1.0),
        // The vector is finite, but its naive squared norm overflows. The
        // old clipping path returned a successful zero instead of a direction.
        b.solve_bfgs_direction(&[vec![1.0]], &[1e200], 1.0),
        // The normal system is finite; the Gauss-Newton norm overflows.
        b.solve_dogleg(&[vec![1.0]], &[1e200], 1.0),
        // Normal-matrix accumulation and damped addition overflow separately.
        b.solve_damped_normal(&[vec![1e154], vec![1e154]], &[1.0, 1.0], 0.0),
        b.solve_damped_normal(&[vec![1e154]], &[1.0], 1e308),
    ] {
        assert_eq!(result.unwrap_err(), LinearSolveError::NonFiniteArithmetic);
    }
    assert_eq!(
        b.solve_damped_normal(&[vec![0.0]], &[1.0], 0.0)
            .unwrap_err(),
        LinearSolveError::Singular
    );
    // Failure has no retained poison: the same backend accepts ordinary work.
    let (step, report) = b.solve_damped_normal(&[vec![2.0]], &[-5.0], 1.0).unwrap();
    assert_eq!(step, [2.0]);
    assert!(report.is_lossy());
    assert_eq!(report.min_abs_pivot, Some(5.0));
}

#[test]
fn ordinary_directions_retain_their_values_and_receipts() {
    let b = DenseLinearBackend;
    for (result, expected) in [
        (b.solve_damped_normal(&[vec![2.0]], &[-4.0], 0.0), 2.0),
        (b.solve_dogleg(&[vec![2.0]], &[-4.0], 3.0), 2.0),
        (b.solve_dogleg(&[vec![2.0]], &[-4.0], 1.0), 1.0),
        (b.solve_bfgs_direction(&[vec![1.0]], &[-2.0], 1.0), 1.0),
    ] {
        let (step, report) = result.unwrap();
        assert_eq!(step[0].to_bits(), f64::to_bits(expected));
        assert!(report.is_lossy());
    }
    for parameter in [0.0, -1.0] {
        assert_eq!(
            b.solve_dogleg(&[vec![1.0]], &[-1.0], parameter).unwrap().0,
            [f64::EPSILON]
        );
        assert_eq!(
            b.solve_bfgs_direction(&[vec![1.0]], &[1.0], parameter)
                .unwrap()
                .0,
            [-f64::EPSILON]
        );
    }
}

#[test]
fn nonlinear_reports_preserve_new_linear_failure_diagnostics() {
    let large = Real::try_from(1e200).unwrap();
    for (engine, offset, reason) in [
        (
            ProposalEngineKind::DampedLeastSquares,
            Real::one(),
            ConvergenceReason::NonFiniteLinearArithmetic,
        ),
        (
            ProposalEngineKind::Dogleg,
            Real::one(),
            ConvergenceReason::NonFiniteLinearArithmetic,
        ),
        (
            ProposalEngineKind::Bfgs,
            large.clone(),
            ConvergenceReason::NonFiniteLinearInput,
        ),
    ] {
        let mut problem = Problem::default();
        problem.add_variable("x", Real::zero());
        problem.add_constraint(Constraint::equality(
            "large affine row",
            Expr::real(large.clone()) * Expr::symbol(SymbolId(0), "x") + Expr::real(offset),
        ));
        let report = solve_damped_least_squares(SolverState {
            problem,
            config: SolverConfig {
                max_iterations: 1,
                proposal_engine: engine,
                ..SolverConfig::default()
            },
        });
        assert_eq!(report.reason, reason);
        assert_eq!(report.iterations, 0);
        assert!(report.linear_reports.is_empty());
        assert_eq!(report.residuals.len(), 1);
    }
}
