//! Generic exact residual replay helpers.
//!
//! Dense numerical solvers and external domain crates may propose a vector for
//! a small linear system, but the proposal is not proof. This module replays
//! `A*x - b` with `hyperreal::Real` and reports exact residual signs before an
//! adapter result can be accepted. That follows Yap's exact-computation rule:
//! represent the mathematical object exactly and make only error-free
//! decisions. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997). The residual formula is deliberately
//! plain dense linear algebra; exact Gaussian/Bareiss solving belongs to
//! [`crate::direct`], while this helper is only the replay boundary.

use hyperreal::{CertifiedRealSign, Real, RealSign};

/// Error returned by dense residual replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DenseResidualReplayError {
    /// Matrix, right-hand side, or candidate dimensions do not match.
    DimensionMismatch,
    /// A residual sign could not be certified within the requested bound.
    UnknownResidual,
}

/// Exact replay evidence for one dense residual row.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseResidualReplayRow {
    /// Row index in the source dense system.
    pub row_index: usize,
    /// Exact residual value `A[row] * x - b[row]`.
    pub residual: Real,
    /// Certified residual sign.
    pub sign: RealSign,
}

/// Exact residual replay report for a dense linear system.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseResidualReplayReport {
    /// Residual vector `A*x - b`.
    pub residuals: Vec<Real>,
    /// Per-row certified residual signs.
    pub rows: Vec<DenseResidualReplayRow>,
    /// True when every residual was certified zero.
    pub accepted: bool,
}

/// Replays a candidate solution through dense exact residuals.
///
/// The function validates matrix/vector dimensions, computes every residual
/// exactly as a `Real`, and asks `Real::certified_sign_until` to prove each
/// sign. Nonzero residuals produce a rejected report, not an error; an error
/// means the input shape is invalid or exact sign certification did not decide.
pub fn replay_dense_linear_residuals(
    matrix: &[Vec<Real>],
    rhs: &[Real],
    candidate: &[Real],
    min_precision: i32,
) -> Result<DenseResidualReplayReport, DenseResidualReplayError> {
    if matrix.len() != rhs.len() || matrix.iter().any(|row| row.len() != candidate.len()) {
        return Err(DenseResidualReplayError::DimensionMismatch);
    }

    let residuals = matrix
        .iter()
        .zip(rhs)
        .map(|(row, rhs)| {
            let mut sum = Real::zero();
            for (coefficient, value) in row.iter().zip(candidate) {
                sum = sum + (coefficient * value);
            }
            sum - rhs
        })
        .collect::<Vec<_>>();

    let mut accepted = true;
    let mut rows = Vec::with_capacity(residuals.len());
    for (row_index, residual) in residuals.iter().enumerate() {
        let sign = match residual.certified_sign_until(min_precision) {
            CertifiedRealSign::Known { sign, .. } => sign,
            CertifiedRealSign::Unknown { .. } => {
                return Err(DenseResidualReplayError::UnknownResidual);
            }
        };
        match sign {
            RealSign::Zero => {}
            RealSign::Negative | RealSign::Positive => accepted = false,
        }
        rows.push(DenseResidualReplayRow {
            row_index,
            residual: residual.clone(),
            sign,
        });
    }

    Ok(DenseResidualReplayReport {
        residuals,
        rows,
        accepted,
    })
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    #[test]
    fn dense_replay_accepts_exact_solution_and_reports_zero_rows() {
        let report = replay_dense_linear_residuals(
            &[vec![real(2), real(1)], vec![real(1), real(-1)]],
            &[real(5), real(1)],
            &[real(2), real(1)],
            -64,
        )
        .unwrap();

        assert!(report.accepted);
        assert_eq!(report.residuals, vec![Real::zero(), Real::zero()]);
        assert_eq!(report.rows.len(), 2);
        assert!(report.rows.iter().all(|row| row.sign == RealSign::Zero));
    }

    #[test]
    fn dense_replay_rejects_nonzero_residual_without_hiding_sign() {
        let report =
            replay_dense_linear_residuals(&[vec![real(3)]], &[real(7)], &[real(2)], -64).unwrap();

        assert!(!report.accepted);
        assert_eq!(report.residuals, vec![real(-1)]);
        assert_eq!(report.rows[0].sign, RealSign::Negative);
    }

    #[test]
    fn dense_replay_rejects_bad_shapes_antagonistically() {
        assert_eq!(
            replay_dense_linear_residuals(&[vec![real(1), real(2)]], &[real(1)], &[real(1)], -64)
                .unwrap_err(),
            DenseResidualReplayError::DimensionMismatch
        );
        assert_eq!(
            replay_dense_linear_residuals(&[vec![real(1)]], &[real(1), real(2)], &[real(1)], -64)
                .unwrap_err(),
            DenseResidualReplayError::DimensionMismatch
        );
    }

    proptest! {
        #[test]
        fn generated_diagonal_systems_replay_exactly(
            a in 1_i16..=32,
            b in 1_i16..=32,
            x in -64_i16..=64,
            y in -64_i16..=64,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let x = i64::from(x);
            let y = i64::from(y);
            let report = replay_dense_linear_residuals(
                &[vec![real(a), Real::zero()], vec![Real::zero(), real(b)]],
                &[real(a * x), real(b * y)],
                &[real(x), real(y)],
                -64,
            ).unwrap();

            prop_assert!(report.accepted);
            prop_assert_eq!(report.rows.len(), 2);
            prop_assert!(report.rows.iter().all(|row| row.sign == RealSign::Zero));
        }
    }
}
