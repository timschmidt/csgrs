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

use std::collections::BTreeMap;

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

/// Prepared sparse linear residual system for repeated exact replay.
///
/// This object is the first sparse batch-assembly boundary for large-sketch and
/// domain-stamp callers. It validates the declared sparse matrix shape once,
/// accumulates duplicate `(row, column)` terms exactly, and then reuses that
/// assembled row structure for one or more candidate vectors. Following Yap,
/// "Towards Exact Geometric Computation" (1997), the prepared object is only a
/// retained exact construction; each replay still certifies `A*x-b` before a
/// candidate can be accepted.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedSparseLinearSystem {
    row_count: usize,
    column_count: usize,
    row_terms: Vec<Vec<(usize, Real)>>,
    rhs: Vec<Real>,
}

impl PreparedSparseLinearSystem {
    /// Number of residual rows in the prepared sparse system.
    pub const fn row_count(&self) -> usize {
        self.row_count
    }

    /// Number of candidate coordinates consumed by the prepared sparse system.
    pub const fn column_count(&self) -> usize {
        self.column_count
    }

    /// Exact right-hand side vector retained with the prepared system.
    pub fn rhs(&self) -> &[Real] {
        &self.rhs
    }

    /// Row-wise exact sparse terms after duplicate accumulation.
    pub fn row_terms(&self) -> &[Vec<(usize, Real)>] {
        &self.row_terms
    }

    /// Replay one candidate vector through the prepared sparse system.
    pub fn replay_candidate(
        &self,
        candidate: &[Real],
        min_precision: i32,
    ) -> Result<SparseResidualReplayReport, SparseResidualReplayError> {
        if candidate.len() != self.column_count {
            return Err(SparseResidualReplayError::DimensionMismatch);
        }
        replay_prepared_sparse_rows(self, candidate, min_precision)
    }

    /// Replay candidates in deterministic input order.
    ///
    /// The work is independent per candidate, so a future parallel backend can
    /// split it. The report order and row probes are part of this API contract
    /// and must remain deterministic.
    pub fn replay_batch(
        &self,
        candidates: &[Vec<Real>],
        min_precision: i32,
    ) -> Result<SparseResidualBatchReport, SparseResidualReplayError> {
        replay_sparse_linear_residual_batch_prepared(self, candidates, min_precision)
    }
}

/// Batch-level status for one prepared sparse replay candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SparseResidualBatchStatus {
    /// Every sparse residual row was certified zero.
    Accepted,
    /// At least one sparse residual row was certified nonzero.
    Rejected,
}

/// Deterministic replay report for one sparse candidate vector.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseResidualBatchReplay {
    /// Candidate ordinal from the caller-supplied input order.
    pub candidate_index: usize,
    /// Exact sparse residual replay for this candidate.
    pub replay: SparseResidualReplayReport,
    /// Candidate-level status derived from the row signs.
    pub status: SparseResidualBatchStatus,
    /// First row that was certified nonzero, if any.
    pub first_nonzero_row: Option<usize>,
    /// All rows certified nonzero.
    pub nonzero_rows: Vec<usize>,
}

/// Deterministic batch report for a prepared sparse linear system.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseResidualBatchReport {
    /// Per-candidate reports in input order.
    pub candidates: Vec<SparseResidualBatchReplay>,
    /// Number of candidate vectors examined.
    pub candidate_count: usize,
    /// Number of candidates certified as exact sparse solutions.
    pub accepted_candidates: usize,
    /// Number of candidates rejected by certified nonzero residual rows.
    pub rejected_candidates: usize,
}

impl SparseResidualBatchReport {
    /// Returns true when at least one candidate satisfies every sparse row.
    pub fn has_accepted_candidate(&self) -> bool {
        self.accepted_candidates > 0
    }
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
    let prepared = prepare_sparse_linear_residual_system(row_count, column_count, terms, rhs)?;
    prepared.replay_candidate(candidate, min_precision)
}

/// Prepare a sparse linear residual system for repeated exact replay.
///
/// Terms are validated against the declared shape and duplicate entries are
/// accumulated exactly into row-local sparse lists. This mirrors sparse matrix
/// assembly in network and constraint systems while keeping the proof boundary
/// at residual certification, as required by Yap's exact-geometric-computation
/// model.
pub fn prepare_sparse_linear_residual_system(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    rhs: &[Real],
) -> Result<PreparedSparseLinearSystem, SparseResidualReplayError> {
    if rhs.len() != row_count {
        return Err(SparseResidualReplayError::DimensionMismatch);
    }

    let mut assembled = BTreeMap::<(usize, usize), Real>::new();
    for term in terms {
        if term.row >= row_count || term.column >= column_count {
            return Err(SparseResidualReplayError::TermOutOfBounds {
                row: term.row,
                column: term.column,
            });
        }
        let entry = assembled
            .entry((term.row, term.column))
            .or_insert_with(Real::zero);
        *entry = entry.clone() + term.coefficient.clone();
    }

    let mut row_terms = vec![Vec::new(); row_count];
    for ((row, column), coefficient) in assembled {
        row_terms[row].push((column, coefficient));
    }

    Ok(PreparedSparseLinearSystem {
        row_count,
        column_count,
        row_terms,
        rhs: rhs.to_vec(),
    })
}

/// Replay a sparse linear system for a batch of candidate vectors.
///
/// This prepares the sparse system once, then replays every candidate in stable
/// input order. It is the public convenience wrapper for callers that do not
/// need to keep the prepared sparse object.
pub fn replay_sparse_linear_residual_batch(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    rhs: &[Real],
    candidates: &[Vec<Real>],
    min_precision: i32,
) -> Result<SparseResidualBatchReport, SparseResidualReplayError> {
    let prepared = prepare_sparse_linear_residual_system(row_count, column_count, terms, rhs)?;
    replay_sparse_linear_residual_batch_prepared(&prepared, candidates, min_precision)
}

/// Replay candidates through a preassembled sparse linear system.
pub fn replay_sparse_linear_residual_batch_prepared(
    prepared: &PreparedSparseLinearSystem,
    candidates: &[Vec<Real>],
    min_precision: i32,
) -> Result<SparseResidualBatchReport, SparseResidualReplayError> {
    let mut replays = Vec::with_capacity(candidates.len());
    for (candidate_index, candidate) in candidates.iter().enumerate() {
        let replay = prepared.replay_candidate(candidate, min_precision)?;
        replays.push(sparse_batch_replay_from_report(candidate_index, replay));
    }

    let accepted_candidates = replays
        .iter()
        .filter(|candidate| candidate.status == SparseResidualBatchStatus::Accepted)
        .count();
    let rejected_candidates = replays
        .iter()
        .filter(|candidate| candidate.status == SparseResidualBatchStatus::Rejected)
        .count();

    Ok(SparseResidualBatchReport {
        candidate_count: replays.len(),
        candidates: replays,
        accepted_candidates,
        rejected_candidates,
    })
}

fn replay_prepared_sparse_rows(
    prepared: &PreparedSparseLinearSystem,
    candidate: &[Real],
    min_precision: i32,
) -> Result<SparseResidualReplayReport, SparseResidualReplayError> {
    let mut residuals = vec![Real::zero(); prepared.row_count];
    for (row_index, row_terms) in prepared.row_terms.iter().enumerate() {
        for (column, coefficient) in row_terms {
            residuals[row_index] =
                residuals[row_index].clone() + (coefficient * &candidate[*column]);
        }
    }
    for (residual, rhs) in residuals.iter_mut().zip(&prepared.rhs) {
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

fn sparse_batch_replay_from_report(
    candidate_index: usize,
    replay: SparseResidualReplayReport,
) -> SparseResidualBatchReplay {
    let nonzero_rows = replay
        .rows
        .iter()
        .filter(|row| row.sign != RealSign::Zero)
        .map(|row| row.row_index)
        .collect::<Vec<_>>();
    let first_nonzero_row = nonzero_rows.first().copied();
    let status = if replay.accepted {
        SparseResidualBatchStatus::Accepted
    } else {
        SparseResidualBatchStatus::Rejected
    };

    SparseResidualBatchReplay {
        candidate_index,
        replay,
        status,
        first_nonzero_row,
        nonzero_rows,
    }
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

    #[test]
    fn prepared_sparse_batch_replay_preserves_order_and_failed_row_probes() {
        let prepared = prepare_sparse_linear_residual_system(
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
        )
        .unwrap();

        assert_eq!(prepared.row_count(), 2);
        assert_eq!(prepared.column_count(), 2);
        assert_eq!(prepared.row_terms()[0].len(), 2);
        assert_eq!(prepared.row_terms()[0][0], (0, real(2)));
        assert_eq!(prepared.rhs(), &[real(5), real(1)]);

        let report = prepared
            .replay_batch(
                &[
                    vec![real(2), real(1)],
                    vec![real(1), real(1)],
                    vec![real(3), real(0)],
                ],
                -64,
            )
            .unwrap();

        assert_eq!(report.candidate_count, 3);
        assert_eq!(report.accepted_candidates, 1);
        assert_eq!(report.rejected_candidates, 2);
        assert!(report.has_accepted_candidate());
        assert_eq!(report.candidates[0].candidate_index, 0);
        assert_eq!(
            report.candidates[0].status,
            SparseResidualBatchStatus::Accepted
        );
        assert_eq!(report.candidates[1].candidate_index, 1);
        assert_eq!(report.candidates[1].first_nonzero_row, Some(0));
        assert_eq!(report.candidates[1].nonzero_rows, vec![0, 1]);
        assert_eq!(report.candidates[2].candidate_index, 2);
        assert_eq!(
            report.candidates[2].status,
            SparseResidualBatchStatus::Rejected
        );
    }

    #[test]
    fn prepared_sparse_batch_rejects_bad_candidate_shapes() {
        let prepared = prepare_sparse_linear_residual_system(1, 2, &[], &[real(0)]).unwrap();

        assert_eq!(
            prepared
                .replay_batch(&[vec![real(0)], vec![real(0), real(0)]], -64)
                .unwrap_err(),
            SparseResidualReplayError::DimensionMismatch
        );
        assert_eq!(
            prepare_sparse_linear_residual_system(1, 1, &[], &[real(0), real(1)]).unwrap_err(),
            SparseResidualReplayError::DimensionMismatch
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

        #[test]
        fn generated_prepared_sparse_batches_preserve_candidate_status(
            a in 1_i16..=32,
            b in 1_i16..=32,
            x in -64_i16..=64,
            y in -64_i16..=64,
            dx in -3_i16..=3,
            dy in -3_i16..=3,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let x = i64::from(x);
            let y = i64::from(y);
            let dx = i64::from(dx);
            let dy = i64::from(dy);
            let prepared = prepare_sparse_linear_residual_system(
                2,
                2,
                &[
                    SparseResidualTerm { row: 0, column: 0, coefficient: real(a) },
                    SparseResidualTerm { row: 1, column: 1, coefficient: real(b) },
                ],
                &[real(a * x), real(b * y)],
            ).unwrap();
            let report = prepared.replay_batch(
                &[
                    vec![real(x), real(y)],
                    vec![real(x + dx), real(y + dy)],
                ],
                -64,
            ).unwrap();

            prop_assert_eq!(report.candidate_count, 2);
            prop_assert_eq!(report.candidates[0].status, SparseResidualBatchStatus::Accepted);
            let second_accepted = dx == 0 && dy == 0;
            prop_assert_eq!(
                report.candidates[1].status == SparseResidualBatchStatus::Accepted,
                second_accepted
            );
            prop_assert_eq!(report.accepted_candidates, if second_accepted { 2 } else { 1 });
        }
    }
}
