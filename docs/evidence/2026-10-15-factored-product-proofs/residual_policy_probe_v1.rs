use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::{Real, RealSign};
use hypersolve::residual_replay::{DenseResidualReplayError, replay_dense_linear_residuals};

fn main() {
    for first_policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let e = Real::e();
        let pi = Real::pi();
        let determinant = &e * &e - &pi * &pi;
        let matrix = [vec![e.clone(), pi.clone()], vec![pi.clone(), e.clone()]];
        let rhs = [Real::from(1), Real::from(2)];
        let candidate = [
            ((e.clone() - Real::from(2) * &pi) / determinant.clone()).unwrap(),
            ((Real::from(2) * &e - pi) / determinant).unwrap(),
        ];
        for policy in [
            first_policy,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let report =
                replay_dense_linear_residuals(&matrix, &rhs, &candidate, -128, policy)
                    .unwrap();
            assert!(report.accepted);
            assert_eq!(report.certainty, Certainty::Exact);
            assert_eq!(report.rows.len(), 2);
            for row in &report.rows {
                assert_eq!(row.sign, RealSign::Zero);
                assert_eq!(row.certainty, Certainty::Exact);
            }
        }
    }
    println!("original system: exact acceptance under both policies, cold and reused");
    for terminal_row in 0..2 {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let mut candidate = [Real::zero(), Real::zero()];
        candidate[terminal_row] = &sine * &sine + &cosine * &cosine - Real::one();
        let matrix = [
            vec![Real::one(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        let rhs = [Real::zero(), Real::zero()];
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let result =
                replay_dense_linear_residuals(&matrix, &rhs, &candidate, -128, policy);
            if policy == PredicatePolicy::STRICT {
                assert_eq!(result.unwrap_err(), DenseResidualReplayError::UnknownResidual);
            } else {
                let report = result.unwrap();
                assert!(report.accepted);
                assert_eq!(report.certainty, Certainty::Approximate);
                assert_eq!(report.rows.len(), 2);
                for (index, row) in report.rows.iter().enumerate() {
                    assert_eq!(row.sign, RealSign::Zero);
                    assert_eq!(
                        row.certainty,
                        if index == terminal_row {
                            Certainty::Approximate
                        } else {
                            Certainty::Exact
                        }
                    );
                }
            }
        }
    }
    println!(
        "trigonometric residual: strict Unknown, qualified approximate zero, strict Unknown; both row orders"
    );
}
