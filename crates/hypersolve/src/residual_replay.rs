//! Generic exact residual replay helpers.
//!
//! Dense and sparse numerical solvers and external domain crates may propose a
//! vector for a small linear system, but the proposal is not proof. This module
//! replays `A*x - b` with `hyperreal::Real` and reports exact residual signs
//! before an adapter result can be accepted. That follows Yap's
//! exact-computation rule: represent the mathematical object exactly and make
//! only error-free decisions. See Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997). The residual formula is deliberately
//! plain linear algebra; exact Gaussian/Bareiss solving belongs to
//! [`crate::direct`], while these helpers are only replay boundaries.

use hyperreal::{CertifiedRealSign, Real, RealSign};

/// Error returned by dense residual replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DenseResidualReplayError {
    /// Matrix, right-hand side, or candidate dimensions do not match.
    DimensionMismatch,
    /// A residual sign could not be certified within the requested bound.
    UnknownResidual,
}

/// Error returned by sparse residual replay.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SparseResidualReplayError {
    /// Matrix shape, right-hand side, or candidate dimensions do not match.
    DimensionMismatch,
    /// A sparse term addressed a row or column outside the declared shape.
    TermOutOfBounds {
        /// Offending term row.
        row: usize,
        /// Offending term column.
        column: usize,
    },
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

/// One exact sparse coefficient in a linear residual system.
///
/// Terms encode `A[row, column] += coefficient`. Repeated `(row, column)`
/// entries are valid and are accumulated exactly during replay, which matches
/// sparse assembly workflows such as MNA stamping while preserving Yap's
/// construction/proof split at the final residual sign decision.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseResidualTerm {
    /// Row receiving this coefficient.
    pub row: usize,
    /// Candidate coordinate multiplied by this coefficient.
    pub column: usize,
    /// Exact coefficient value.
    pub coefficient: Real,
}

/// Exact replay evidence for one sparse residual row.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseResidualReplayRow {
    /// Row index in the source sparse system.
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

/// Exact residual replay report for a sparse linear system.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseResidualReplayReport {
    /// Residual vector `A*x - b`.
    pub residuals: Vec<Real>,
    /// Per-row certified residual signs.
    pub rows: Vec<SparseResidualReplayRow>,
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

/// Replays a candidate solution through sparse exact residuals.
///
/// The declared shape is validated before any proof decision is made. Sparse
/// terms are then accumulated exactly into `row_count` residual slots, the
/// right-hand side is subtracted, and each residual sign is certified through
/// `Real::certified_sign_until`. This keeps sparse numeric solvers, fill
/// reducers, and domain-specific stamp assemblers outside the proof boundary:
/// they may propose `x`, but this replay report decides whether `A*x-b` is
/// actually zero. This is the same construction/proof separation emphasized by
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
pub fn replay_sparse_linear_residuals(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    rhs: &[Real],
    candidate: &[Real],
    min_precision: i32,
) -> Result<SparseResidualReplayReport, SparseResidualReplayError> {
    if rhs.len() != row_count || candidate.len() != column_count {
        return Err(SparseResidualReplayError::DimensionMismatch);
    }

    let mut residuals = vec![Real::zero(); row_count];
    for term in terms {
        if term.row >= row_count || term.column >= column_count {
            return Err(SparseResidualReplayError::TermOutOfBounds {
                row: term.row,
                column: term.column,
            });
        }
        residuals[term.row] =
            residuals[term.row].clone() + (&term.coefficient * &candidate[term.column]);
    }
    for (residual, rhs) in residuals.iter_mut().zip(rhs) {
        *residual = residual.clone() - rhs;
    }

    let mut accepted = true;
    let mut rows = Vec::with_capacity(residuals.len());
    for (row_index, residual) in residuals.iter().enumerate() {
        let sign = match residual.certified_sign_until(min_precision) {
            CertifiedRealSign::Known { sign, .. } => sign,
            CertifiedRealSign::Unknown { .. } => {
                return Err(SparseResidualReplayError::UnknownResidual);
            }
        };
        match sign {
            RealSign::Zero => {}
            RealSign::Negative | RealSign::Positive => accepted = false,
        }
        rows.push(SparseResidualReplayRow {
            row_index,
            residual: residual.clone(),
            sign,
        });
    }

    Ok(SparseResidualReplayReport {
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

    #[test]
    fn sparse_replay_accepts_exact_solution_and_duplicate_terms() {
        let report = replay_sparse_linear_residuals(
            2,
            2,
            &[
                SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 0,
                    column: 1,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 1,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 1,
                    column: 1,
                    coefficient: real(-1),
                },
            ],
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
    fn sparse_replay_rejects_nonzero_residual_without_hiding_sign() {
        let report = replay_sparse_linear_residuals(
            1,
            1,
            &[SparseResidualTerm {
                row: 0,
                column: 0,
                coefficient: real(3),
            }],
            &[real(7)],
            &[real(2)],
            -64,
        )
        .unwrap();

        assert!(!report.accepted);
        assert_eq!(report.residuals, vec![real(-1)]);
        assert_eq!(report.rows[0].sign, RealSign::Negative);
    }

    #[test]
    fn sparse_replay_rejects_bad_shapes_antagonistically() {
        assert_eq!(
            replay_sparse_linear_residuals(1, 2, &[], &[real(1)], &[real(1)], -64).unwrap_err(),
            SparseResidualReplayError::DimensionMismatch
        );
        assert_eq!(
            replay_sparse_linear_residuals(1, 1, &[], &[real(1), real(2)], &[real(1)], -64)
                .unwrap_err(),
            SparseResidualReplayError::DimensionMismatch
        );
        assert_eq!(
            replay_sparse_linear_residuals(
                1,
                1,
                &[SparseResidualTerm {
                    row: 1,
                    column: 0,
                    coefficient: real(1),
                }],
                &[real(0)],
                &[real(0)],
                -64,
            )
            .unwrap_err(),
            SparseResidualReplayError::TermOutOfBounds { row: 1, column: 0 }
        );
        assert_eq!(
            replay_sparse_linear_residuals(
                1,
                1,
                &[SparseResidualTerm {
                    row: 0,
                    column: 1,
                    coefficient: real(1),
                }],
                &[real(0)],
                &[real(0)],
                -64,
            )
            .unwrap_err(),
            SparseResidualReplayError::TermOutOfBounds { row: 0, column: 1 }
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

        #[test]
        fn generated_sparse_diagonal_systems_replay_exactly(
            a in 1_i16..=32,
            b in 1_i16..=32,
            x in -64_i16..=64,
            y in -64_i16..=64,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let x = i64::from(x);
            let y = i64::from(y);
            let report = replay_sparse_linear_residuals(
                2,
                2,
                &[
                    SparseResidualTerm { row: 0, column: 0, coefficient: real(a) },
                    SparseResidualTerm { row: 1, column: 1, coefficient: real(b) },
                ],
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
